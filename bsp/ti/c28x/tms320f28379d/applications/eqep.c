#include "board.h"

#define PIN_EQEP_A  24
#define PIN_EQEP_B  25
#define PIN_EQEP_I  26

static volatile uint32_t qep_old_position;
static float qep_speed_float,qep_total_counts_float,qep_speed_coef_float,qep_angle_coef_float;
static int32_t qep_total_counts_int32,qep_half_counts_int32,qep_half_counts_int32_minus;

interrupt void qposcnt_latch_isr(void)
{
    int32_t pos_diff;

    if(EQep2Regs.QFLG.bit.PCE)
        goto exit;  //skip speed calculation at position error

    pos_diff = (int32_t)(EQep2Regs.QPOSLAT - qep_old_position);

    if(pos_diff > qep_half_counts_int32)
    {
        pos_diff -= qep_total_counts_int32;
    }
    else if(pos_diff < qep_half_counts_int32_minus)
    {
        pos_diff += qep_total_counts_int32;
    }

    qep_speed_float = (float)pos_diff * qep_speed_coef_float;

exit:
    qep_old_position = EQep2Regs.QPOSLAT;

    EALLOW;
    EQep2Regs.QCLR.all = 0xFFFF;    //clear all flags
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    EDIS;
}

void eqep_setup(float Ts)
{

    qep_speed_float = 0.0;
    qep_old_position = EQep2Regs.QPOSCNT; // do not reset the counter, but keep it

    qep_total_counts_int32 = (int32_t)(1024*4);
    qep_half_counts_int32 = (int32_t)(qep_total_counts_int32/2);
    qep_half_counts_int32_minus = -qep_half_counts_int32;

    qep_total_counts_float = (float)(qep_total_counts_int32);
    qep_angle_coef_float = 1.0 / qep_total_counts_float;            //mech angle, pu
    qep_speed_coef_float = qep_angle_coef_float / Ts;               //mech speed, Hz

    EQep2Regs.QUTMR = 0;
    EQep2Regs.QUPRD = 200000000*Ts;       // 200000 Unit Timer for 1000Hz at 200 MHz SYSCLKOUT
    EQep2Regs.QDECCTL.bit.QSRC = 0;       // QEP quadrature count mode
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep2Regs.QEPCTL.bit.PCRM = 0;        // PCRM=00 mode - QPOSCNT reset on index event
    EQep2Regs.QPOSMAX = qep_total_counts_int32-1;         // 2000 line
    EQep2Regs.QCAPCTL.bit.UPPS = 0;       // 1/2^n for unit position event
    EQep2Regs.QCAPCTL.bit.CCPS = 0;       // 1/2^n for CAP clock
    EQep2Regs.QCAPCTL.bit.CEN = 0;        // QEP Capture Disable
    EQep2Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
    EQep2Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    EQep2Regs.QEINT.bit.UTO = 1;          // Unit Timeout Interrupt Enable
    EQep2Regs.QEINT.bit.IEL = 0;          // Index Interrupt Disable
    EQep2Regs.QCLR.all =  0xFFFF;         // Clear All Interrupt Flags
    EQep2Regs.QEPCTL.bit.QPEN = 1;        // QEP enable

    EALLOW;
    PieVectTable.EQEP2_INT = &qposcnt_latch_isr;
    PieCtrlRegs.PIEIER5.bit.INTx2 = 1;
    IER |= M_INT5;
    EDIS;
    gpio_config(PIN_EQEP_A,2,GPIO_MUX_CPU1,GPIO_LOW,GPIO_INPUT,GPIO_SYNC);
    gpio_config(PIN_EQEP_B,2,GPIO_MUX_CPU1,GPIO_LOW,GPIO_INPUT,GPIO_SYNC);
    gpio_config(PIN_EQEP_I,2,GPIO_MUX_CPU1,GPIO_LOW,GPIO_INPUT,GPIO_SYNC);
}

float eqep_get_angle()
{
    return((float)EQep2Regs.QPOSCNT * qep_angle_coef_float);
}

float eqep_get_speed()
{
    return(qep_speed_float);
}


