#include "board.h"

#define PIN_INV_EN 26

extern interrupt void main_isr(void);
static void pwm_setup(float);
static void adc_setup(void);

union adc_union
{
    uint32_t all;
    struct 
    {
        uint16_t channel:4;
        uint16_t soc:4;
        uint16_t trigger:5;
        uint16_t adcx:3;
        uint16_t ACQPS:8;
        uint16_t protect:2;
        uint16_t resrv:6;
    } bit;
};

typedef struct 
{
    union adc_union adc_set;
    int16_t th_low;
    int16_t th_high;
} adc_param_t;

union pwm_union {
    uint32_t  all;
    struct {
        uint16_t blank:7;
        uint16_t hspdiv:3;
        uint16_t div:3;
        uint16_t trigger_B:3;
        uint16_t trigger_A:3;
        uint16_t data_type:2;
        uint16_t input:3;
        uint16_t mode:2;
        uint16_t sync:2;
        uint16_t pwmx:4;
    } bit;
};

typedef struct {
    union pwm_union pwm_set;
    uint16_t base_period;
    uint16_t dead_time;
    uint16_t default_phase;
    uint16_t default_duty;
    float period_pu;
    float duty_pu;
    float phase_pu;
} pwm_param_t;

int16_t pwm_pin_table[] = {
    -1,        //EPWM0
    -1,        //EPWM1
    -1,        //EPWM2
    -1,        //EPWM3
    6,         //EPWM4
    8,         //EPWM5
    10,        //EPWM6
};

int16_t pwm_mux_table[] = {
    -1,        //EPWM0
    1,         //EPWM1
    1,         //EPWM2
    1,         //EPWM3
    1,         //EPWM4
    1,         //EPWM5
    1,         //EPWM6
};

int16_t adc_chn_table[] = {
    4,         //ADCA
    4,         //ADCB
    4,         //ADCC
    15,        //ADCD 
};

#define PWM_PIN(x) EPWM##x

#define SOC_CONFIG(adc_reg, param, soc_num) \
    do { \
        adc_reg->ADCSOC##soc_num##CTL.bit.CHSEL = param->adc_set.bit.channel; \
        adc_reg->ADCSOC##soc_num##CTL.bit.ACQPS = param->adc_set.bit.ACQPS; \
        adc_reg->ADCSOC##soc_num##CTL.bit.TRIGSEL = param->adc_set.bit.trigger + 4; \
    } while(0)  

#define PPB_CONFIG(adc_reg, param, ppb_num) \
    do { \
        adc_reg->ADCPPB##ppb_num##CONFIG.bit.CONFIG = (ppb_num)-1; \
        adc_reg->ADCPPB##ppb_num##OFFREF = 0; \
        adc_reg->ADCEVTSEL.bit.PPB##ppb_num##TRIPHI = param->adc_set.bit.protect & 0x1; \
        adc_reg->ADCEVTSEL.bit.PPB##ppb_num##TRIPLO = param->adc_set.bit.protect >> 1; \
        adc_reg->ADCEVTINTSEL.bit.PPB##ppb_num##TRIPHI = param->adc_set.bit.protect & 0x1; \
        adc_reg->ADCEVTINTSEL.bit.PPB##ppb_num##TRIPLO = param->adc_set.bit.protect >> 1; \
        adc_reg->ADCPPB##ppb_num##TRIPHI.bit.LIMITHI = param->th_high; \
        adc_reg->ADCPPB##ppb_num##TRIPLO.bit.LIMITLO = param->th_low; \
    } while(0)

inline void inv_enable(int16_t status)
{
    /* 0: enable; 1: disable */
    GPIO_WritePin(PIN_INV_EN,status);
}

interrupt void adca_evt_isr(void)
{
    inv_enable(1);
}

interrupt void adcb_evt_isr(void)
{
    inv_enable(1);
}

interrupt void adcc_evt_isr(void)
{
    inv_enable(1);
}

interrupt void adcd_evt_isr(void)
{
    inv_enable(1);
}

void inv_set_duty(abc_dq_t * voltage, float period)
{
    float abc_min;
    float abc_max;
    float abc_shf;

    abc_max = __fmax(voltage->a,voltage->b);
    abc_max = __fmax(abc_max,voltage->c);

    abc_min = __fmax(voltage->a,voltage->b);
    abc_min = __fmax(abc_min,voltage->c);

    abc_shf = -(abc_max + abc_min)/2.0 + 1.0;

    period = period / 2.0;

    EPwm4Regs.CMPA.bit.CMPA = (voltage->a + abc_shf) * period;
    EPwm5Regs.CMPA.bit.CMPA = (voltage->b + abc_shf) * period;
    EPwm6Regs.CMPA.bit.CMPA = (voltage->c + abc_shf) * period;
}

void inv_get_current(abc_dq_t * current)
{
    current->a = ((int16_t)(0x08C4 - AdccResultRegs.ADCRESULT0))/800.0;
    current->b = ((int16_t)(0x08C6 - AdcbResultRegs.ADCRESULT0))/800.0;
    current->c = ((int16_t)(0x08C0 - AdcaResultRegs.ADCRESULT0))/800.0;
}

void inv_setup(float Ts)
{
    adc_setup();
    pwm_setup(Ts);

    gpio_config(PIN_INV_EN,0,GPIO_MUX_CPU1,GPIO_HIGH,GPIO_OUTPUT,GPIO_SYNC);

    EALLOW;
    PieVectTable.ADCA1_INT = &main_isr;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    IER |= M_INT1;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    inv_enable(0);
}

static void pwm_config(pwm_param_t * param){

    volatile struct EPWM_REGS *pwm_reg = &EPwm1Regs + param->pwm_set.bit.pwmx - 1;
    int16_t pwmA_pin,pwmB_pin,pwm_mux;

    //#1: synchronize and phase
    if(param->pwm_set.bit.pwmx == 1){               //PWM1 is the sync source.
        EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;               //PWM1 generates sync pulse when CTR = zero.
        SyncSocRegs.SYNCSELECT.bit.EPWM4SYNCIN = 0;     //select PWM1 as Sync Input Source for EPWM4:
        SyncSocRegs.SYNCSELECT.bit.EPWM7SYNCIN = 0;     //select PWM1 as Sync Input Source for EPWM7:
        SyncSocRegs.SYNCSELECT.bit.EPWM10SYNCIN = 0;    //select PWM1 as Sync Input Source for EPWM10:
        pwm_reg->TBPHS.bit.TBPHS = 0;
    }
    else if(param->pwm_set.bit.sync == 0){
        pwm_reg->TBCTL.bit.PHSEN = 0x0;
    }
    else if(param->pwm_set.bit.sync == 1){
        if((int16_t)param->default_phase >= 0){
            pwm_reg->TBCTL.bit.PHSDIR = 1;              //count-up, leading
        }
        else{
            pwm_reg->TBCTL.bit.PHSDIR = 0;              //count-down, lagging
        }
        pwm_reg->TBPHS.bit.TBPHS = abs((int16_t)param->default_phase);
        pwm_reg->TBCTL.bit.PHSEN = 0x1;                 //enable phase shift
    }

    //#2: counter mode
    pwm_reg->TBCTL.bit.CTRMODE = 0x2;                   //up-down count mode.

    //#3: output mode: normal or invert.
    if(param->pwm_set.bit.mode == 1){                   //normal
        pwm_reg->AQCTLA.bit.CAU = AQ_CLEAR;
        pwm_reg->AQCTLA.bit.CAD = AQ_SET;
        pwm_reg->AQCTLB.bit.CAU = AQ_SET;
        pwm_reg->AQCTLB.bit.CAD = AQ_CLEAR;
    }
    else if(param->pwm_set.bit.mode == 2){              //inverted
        pwm_reg->AQCTLA.bit.CAU = AQ_SET;
        pwm_reg->AQCTLA.bit.CAD = AQ_CLEAR;
        pwm_reg->AQCTLB.bit.CAU = AQ_CLEAR;
        pwm_reg->AQCTLB.bit.CAD = AQ_SET;
    }

    //#4: clock division
    pwm_reg->TBCTL.bit.CLKDIV = param->pwm_set.bit.div;
    pwm_reg->TBCTL.bit.HSPCLKDIV = param->pwm_set.bit.hspdiv;

    //#5: dead-time
    pwm_reg->DBCTL.bit.POLSEL = 2;                  //EPWMxB is inverted
    pwm_reg->DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;   //both RED and FED active
    pwm_reg->DBCTL.bit.IN_MODE = 0;                 //EPWMxA is the source
    pwm_reg->DBRED.bit.DBRED = param->dead_time;
    pwm_reg->DBFED.bit.DBFED = param->dead_time;

    //#6: time base
    pwm_reg->TBPRD = param->base_period;            //Set timer period
    pwm_reg->TBCTR = 0x0000;                        //Clear the counter

    //#7: compare
    pwm_reg->CMPA.bit.CMPA = param->default_duty;

    //#8: set trigger output
    if((param->pwm_set.bit.trigger_A < 4) && (param->pwm_set.bit.trigger_A > 0)) {
        pwm_reg->ETSEL.bit.SOCASEL = param->pwm_set.bit.trigger_A;    // 1:zero; 2: top; 3: zero & top
        pwm_reg->ETPS.bit.SOCAPRD = 1;                      // Generate pulse on 1st event
        pwm_reg->ETSEL.bit.SOCAEN = 1;                      // enable pwmA trigger SOC
    }
    if((param->pwm_set.bit.trigger_B < 4) && (param->pwm_set.bit.trigger_B > 0)) {
        pwm_reg->ETSEL.bit.SOCBSEL = param->pwm_set.bit.trigger_B;
        pwm_reg->ETPS.bit.SOCBPRD = 1;
        pwm_reg->ETSEL.bit.SOCBEN = 1;
    }

    //#9: set digital comparator and trip zone
    EALLOW;
    pwm_reg->TZCLR.bit.OST=1;                   // Clear one-shot trip flag
    pwm_reg->DCTRIPSEL.bit.DCAHCOMPSEL=3;       // Select TRIPIN4 (EpwmXBAR, linked with ADC PPB) as the source for DCAH
    pwm_reg->TZDCSEL.bit.DCAEVT1=2;             // DCAEVT1 is generated when DCAH is high
    pwm_reg->DCACTL.bit.EVT1SRCSEL=0;           // Source signal is unfiltered.
    pwm_reg->DCACTL.bit.EVT1FRCSYNCSEL=1;       // DC input signal is not synced with TBCLK
    pwm_reg->TZSEL.bit.DCAEVT1=1;               // Enable DCAEVT1 as one-shot-trip source for this ePWM module
    pwm_reg->TZSEL.bit.CBC1=0;                  // Disable TZ3 as cycle-by-cycle trip
    pwm_reg->TZSEL.bit.CBC2=0;                  // Disable TZ3 as cycle-by-cycle trip
    pwm_reg->TZSEL.bit.CBC3=0;                  // Disable TZ3 as cycle-by-cycle trip
    pwm_reg->TZCTL.bit.TZA=2;                   // EPWMxA will be forced low on a trip event.
    pwm_reg->TZCTL.bit.TZB=2;                   // EPWMxB will be forced low on a trip event.
    pwm_reg->ETCLR.all = 0xFFFF;                // Clear all events
    EDIS;

    //#10: set pin mux
    pwmA_pin = pwm_pin_table[param->pwm_set.bit.pwmx];
    pwmB_pin = pwmA_pin+1;
    pwm_mux = pwm_mux_table[param->pwm_set.bit.pwmx];
    GPIO_SetupPinMux(pwmA_pin,0,pwm_mux);
    GPIO_SetupPinMux(pwmB_pin,0,pwm_mux);
}

static void adc_config(adc_param_t * param)
{
    volatile struct ADC_REGS *adc_reg = &AdcaRegs + param->adc_set.bit.adcx - 1;

    EALLOW;

    adc_reg->ADCCTL1.bit.ADCPWDNZ = 1; // Analog circuit power up
    adc_reg->ADCCTL2.bit.PRESCALE = 0b0110; // set CLKDIV = 4, the max freq is 50MHz.
    adc_reg->ADCSOCPRICTL.bit.SOCPRIORITY = 0x10; //10h All SOCs are in high priority mode, arbitrated by SOC number.
    AdcSetMode((param->adc_set.bit.adcx-1), ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //set soc
    switch(param->adc_set.bit.soc){
        case 1 :{       //soc0
            SOC_CONFIG(adc_reg, param, 0); 
            PPB_CONFIG(adc_reg, param, 1);
            adc_reg->ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
            adc_reg->ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
            adc_reg->ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
            break;
        }
        case 2 :{       //soc1
            SOC_CONFIG(adc_reg, param, 1);
            PPB_CONFIG(adc_reg, param, 2);
            adc_reg->ADCINTSEL1N2.bit.INT2SEL = 1; //EOC1 will set INT2 flag
            adc_reg->ADCINTSEL1N2.bit.INT2E = 1;   //enable INT2 flag
            adc_reg->ADCINTFLGCLR.bit.ADCINT2 = 1; //make sure INT2 flag is cleared
            break;
        }
        case 3 :{       //soc2
            SOC_CONFIG(adc_reg, param, 2);
            PPB_CONFIG(adc_reg, param, 3);
            adc_reg->ADCINTSEL3N4.bit.INT3SEL = 2; //EOC2 will set INT3 flag
            adc_reg->ADCINTSEL3N4.bit.INT3E = 1;   //enable INT3 flag
            adc_reg->ADCINTFLGCLR.bit.ADCINT3 = 1; //make sure INT3 flag is cleared
            break;
        }
        case 4 :{       //soc3
            SOC_CONFIG(adc_reg, param, 3);
            PPB_CONFIG(adc_reg, param, 4);
            adc_reg->ADCINTSEL3N4.bit.INT4SEL = 3; //end of SOC3 will set INT4 flag
            adc_reg->ADCINTSEL3N4.bit.INT4E = 1;   //enable INT4 flag
            adc_reg->ADCINTFLGCLR.bit.ADCINT3 = 1; //make sure INT4 flag is cleared
            break;
        }
    }

    EDIS;
}

static void pwm_setup(float Ts)
{
    pwm_param_t param;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    param.pwm_set.bit.sync = 0;
    param.pwm_set.bit.mode = 1; //non-inverted
    param.pwm_set.bit.div = 0;
    param.pwm_set.bit.hspdiv = 0;
    param.pwm_set.bit.trigger_A = 1;
    param.pwm_set.bit.trigger_B = 1;
    param.dead_time = 5;
    param.base_period = 100000000*Ts;
    param.default_phase = 0;
    param.default_duty = 5000;

    int i;
    for (i = 4; i <= 6; i++) {
        param.pwm_set.bit.pwmx = i;
        pwm_config(&param);
    }
}

static void adc_setup(void)
{
    adc_param_t param;

    param.adc_set.bit.ACQPS = 0x15;
    param.adc_set.bit.trigger = 0xB; //PWM4A
    param.adc_set.bit.protect = 0x3;
    param.adc_set.bit.soc = 1;
    param.th_low  = 2048-1500;
    param.th_high = 2048+1500;

    int i;
    for(i = 1; i<=4; i++)
    {
        param.adc_set.bit.adcx = i;
        param.adc_set.bit.channel = adc_chn_table[i-1];        
        adc_config(&param);
    }

    EALLOW;

    PieVectTable.ADCA_EVT_INT = &adca_evt_isr;
    PieVectTable.ADCB_EVT_INT = &adcb_evt_isr;
    PieVectTable.ADCC_EVT_INT = &adcc_evt_isr;
    PieVectTable.ADCD_EVT_INT = &adcd_evt_isr;
    PieCtrlRegs.PIEIER10.bit.INTx1 = 1;  // enable adca event interrupt
    PieCtrlRegs.PIEIER10.bit.INTx5 = 1;  // enable adcb event interrupt
    PieCtrlRegs.PIEIER10.bit.INTx9 = 1;  // enable adcc event interrupt
    PieCtrlRegs.PIEIER10.bit.INTx13 = 1; // enable adcd event interrupt

    IER |= M_INT10;

    EDIS;
}
