/******************************************************************
 �� �� ����     epwm.c
 D S P    ��     DSC28034
 ʹ �� �⣺
 ��     �ã�
 ˵     ����    �ṩepwm.c�ӿڳ�ʼ������
 ---------------------------- ʹ��˵�� ----------------------------
 ����������


 �� ����V1.0.0
 ʱ �䣺2022��8��25��
 �� �ߣ�heyang
 @ mail��support@mail.haawking.com
 ******************************************************************/

#include "epwm.h"

/******************************************************************
 ��������void  InitEPwm1_Gpio(void)
 ��	������
 ����ֵ����
 ��	�ã�����GPIO0��GPIO1��GPIO2��GPIO3���ó�epwmģʽ��
 ******************************************************************/

void InitEPwm1_Gpio(void)
{
	EALLOW;

	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;

	EDIS;
}

/******************************************************************
 ��������void  pwm1_config(void)
 ��	������
 ����ֵ����
 ��	�ã�
 TBCLK = SYSCLKOUT/(HSPCLKDIV*CLKDIV)
 ����pwm1Ϊ���Ķ��뷽ʽ��
 ��������ϻ������¼���ģʽPWM�����ڼ��㹫ʽ=TBCLK*TBPRD,
 PWM1A��ռ�ձ�= CMPA/TBPRD,
 PWM1B��ռ�ձ� = CMPB/TBPRD��
 ���Ķ��뷽ʽPWM�����ڼ��㹫ʽ = 2*TBCLK*TBPRD
 ******************************************************************/

void pwm1_config()
{
	EPwm1Regs.TBPRD = 3750;                                                /*�趨pwm��������2*3750��TBCLKʱ�����ڣ�Ƶ��16K*/
	EPwm1Regs.CMPA.half.CMPA = 1875;                                /*�Ƚ���AΪ1875��TBCLKʱ������,ռ�ձ�1875/3750=50%*/
	EPwm1Regs.CMPB = 1000;                                                  /*�Ƚ���BΪ1000��TBCLKʱ������,ռ�ձȣ�3750-1000��/3750=73.33%*/
	EPwm1Regs.TBCTR = 0x0000;                                             /*���������*/
	EPwm1Regs.TBPHS.half.TBPHS = 0x0000;                           /*��λ�Ĵ������㣬�����÷�ת��������ʱ��*/

//    EPwm1Regs.AQCTLA.bit.ZRO = 0;                                    /*��Сֵ*/
//  	EPwm1Regs.AQCTLA.bit.PRD = 0x10;                               /*��ʱ����������ֵ�����ڼĴ�����ֵ���ʱ��λ*/
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;                        /*���¼���ʱ��ʱ����������ֵ��CMPA�Ĵ�����ֵ���ʱ����*/
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;                             /*���ϼ���ʱ��ʱ����������ֵ��CMPA�Ĵ�����ֵ���ʱ��1*/
	EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;                         /*���¼���ʱ��ʱ����������ֵ��CMPB�Ĵ�����ֵ���ʱ����*/
	EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;                              /*���ϼ���ʱ��ʱ����������ֵ��CMPB�Ĵ�����ֵ���ʱ��1*/

	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;  /*�������¼�����ʽ*/
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;                        /*�����Ĵ���װ����λ�Ĵ�����ֹװ��*/
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;                      /*����������ֵΪ0ʱ���ڼĴ���TBPRDװ��Ӱ�ӼĴ�����ֵ*/
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;              /*ͬ���ź����ѡ�񣬵�ʱ������������0ʱ��ePWMxSYNCO�ź����*/
	EPwm1Regs.TBCTL.bit.SWFSYNC = 0;                                    /*���ǿ��ͬ�����壬д0û��Ч��*/
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm1Regs.ETSEL.bit.INTSEL = ET_DCAEVT1SOC;                /*����*/
	EPwm1Regs.ETSEL.bit.INTEN = 0;                                          /*Disable EPWMx_INT generation*/
	EPwm1Regs.ETSEL.bit.SOCASEL = 2;                                     /*��TB�������������ڵ�ʱ�� EPWMxSOCA ���彫����*/
	EPwm1Regs.ETSEL.bit.SOCAEN = 1;                                       /*ʹ��EPWMxSOCA pulse*/
	EPwm1Regs.ETSEL.bit.SOCBEN = 0;
	EPwm1Regs.ETSEL.bit.SOCBSEL = 0;

	EPwm1Regs.ETPS.bit.SOCAPRD = 1;
}





