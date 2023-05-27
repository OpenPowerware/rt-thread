#include "board.h"

#define MUX_EQEP    1
#define PIN_EQEP_A  20
#define PIN_EQEP_B  21
#define PIN_EQEP_I  99

static volatile uint32_t qep_old_position;
static float qep_speed_float,qep_total_counts_float,qep_speed_coef_float,qep_angle_coef_float;
static int32_t qep_total_counts_int32,qep_half_counts_int32,qep_half_counts_int32_minus;

interrupt void qposcnt_latch_isr(void)
{
    int32_t pos_diff;

    if(EQep1Regs.QFLG.bit.PCE)
        goto exit;  //skip speed calculation at position error

    pos_diff = (int32_t)(EQep1Regs.QPOSLAT - qep_old_position);

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
    qep_old_position = EQep1Regs.QPOSLAT;

    EALLOW;
    EQep1Regs.QCLR.all = 0xFFFF;    //clear all flags
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
    EDIS;
}

void eqep_setup(void)
{
    gpio_config(PIN_EQEP_A,MUX_EQEP,GPIO_MUX_CPU1,GPIO_LOW,GPIO_INPUT,GPIO_SYNC);
    gpio_config(PIN_EQEP_B,MUX_EQEP,GPIO_MUX_CPU1,GPIO_LOW,GPIO_INPUT,GPIO_SYNC);
    gpio_config(PIN_EQEP_I,MUX_EQEP,GPIO_MUX_CPU1,GPIO_LOW,GPIO_INPUT,GPIO_SYNC);

    qep_speed_float = 0.0;
    qep_old_position = EQep1Regs.QPOSCNT; // do not reset the counter, but keep it

    qep_total_counts_int32 = (int32_t)(2000*4);
    qep_half_counts_int32 = (int32_t)(2000*2);
    qep_half_counts_int32_minus = -qep_half_counts_int32;

    qep_total_counts_float = (float)(2000*4);
    qep_angle_coef_float = 1.0 / qep_total_counts_float;            //mech angle, pu
    qep_speed_coef_float = qep_angle_coef_float * (200e6 / 200000); //mech speed, Hz

    EQep1Regs.QUTMR = 0;
    EQep1Regs.QUPRD = 200000;             // 2000000 Unit Timer for 1000Hz at 200 MHz SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC = 0;       // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;
    EQep1Regs.QEPCTL.bit.PCRM = 0;        // PCRM=00 mode - QPOSCNT reset on index event
    EQep1Regs.QPOSMAX = 2000*4-1;         // 2000 line
    EQep1Regs.QCAPCTL.bit.UPPS = 0;       // 1/2^n for unit position event
    EQep1Regs.QCAPCTL.bit.CCPS = 0;       // 1/2^n for CAP clock
    EQep1Regs.QCAPCTL.bit.CEN = 0;        // QEP Capture Disable
    EQep1Regs.QEPCTL.bit.QCLM = 1;        // Latch on unit time out
    EQep1Regs.QEPCTL.bit.UTE = 1;         // Unit Timeout Enable
    EQep1Regs.QEINT.bit.UTO = 1;          // Unit Timeout Interrupt Enable
    EQep1Regs.QEINT.bit.IEL = 0;          // Index Interrupt Disable
    EQep1Regs.QCLR.all =  0xFFFF;         // Clear All Interrupt Flags
    EQep1Regs.QEPCTL.bit.QPEN = 1;        // QEP enable

    EALLOW;
    PieVectTable.EQEP1_INT = &qposcnt_latch_isr;
    PieCtrlRegs.PIEIER5.bit.INTx1 = 1;
    IER |= M_INT5;
    EDIS;
}

float eqep_get_angle()
{
    return((float)EQep1Regs.QPOSCNT * qep_angle_coef_float);
}

float eqep_get_speed()
{
    return(qep_speed_float);
}


