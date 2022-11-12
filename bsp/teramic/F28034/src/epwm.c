/******************************************************************
 �� �� ����     epwm.c
 D S P��       DSC28034
 ʹ �� �⣺     
 ��     �ã�      
 ˵     ����      �ṩepwm.c�ӿڳ�ʼ������
 ---------------------------- ʹ��˵�� ----------------------------
 ����������


 �� ����V1.0.x
 ʱ �䣺2021��11��13��
 �� �ߣ�
 @ mail��support@mail.haawking.com
 ******************************************************************/

#include "epwm.h"

/******************************************************************
 ��������void  pwm1_gpio_init(void)
 ��	������
 ����ֵ����
 ��	�ã�����GPIO0��GPIO1Ϊ��ֹ����ģʽ������GPIO0ΪPWM1A��GPIO1ΪPWM1B
 ******************************************************************/

void pwm1_gpio_init()
{
	EALLOW;

	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1; /* Disable pull-up on GPIO(EPWM1) */

	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1; /* ����GPIO0ΪPWM1A */
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;

	EDIS;
}

/******************************************************************
 ��������void  pwm1_config(void)
 ��	������
 ����ֵ����
 ��	�ã�����pwm1Ϊ��������ģʽ�����ò���epwm1socA�¼���
 ******************************************************************/

void pwm1_config()
{
	EPwm1Regs.TBPRD = 0x599; /* �趨pwm��������1433��TBclkʱ������ */
	EPwm1Regs.CMPA.half.CMPA = 0x200; /* �Ƚ���AΪ512��TBCLKʱ������ */
	EPwm1Regs.CMPB = 0x300; /* �Ƚ���BΪ768��TBCLKʱ������ */
	EPwm1Regs.TBCTR = 0x0000; /* clear counter */
	EPwm1Regs.TBPHS.half.TBPHS = 0x0000; /* ��λ�Ĵ������㣬�����÷�ת��������ʱ�� */

	/* EPwm1Regs.AQCTLA.bit.ZRO = 0;    ��Сֵ */
	/* EPwm1Regs.AQCTLA.bit.PRD = 0x10;   ��ʱ����������ֵ�����ڼĴ�����ֵ���ʱ��λ */
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR; /* ���¼���ʱ��ʱ����������ֵ��CMPA�Ĵ�����ֵ���ʱ���� */
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; /* ���ϼ���ʱ��ʱ����������ֵ��CMPA�Ĵ�����ֵ���ʱ��1 */
	EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR; /* ���¼���ʱ��ʱ����������ֵ��CMPB�Ĵ�����ֵ���ʱ���� */
	EPwm1Regs.AQCTLB.bit.CBU = AQ_SET; /* ���ϼ���ʱ��ʱ����������ֵ��CMPB�Ĵ�����ֵ���ʱ��1 */

	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; /* �������¼�����ʽ */
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; /* �����Ĵ���װ����λ�Ĵ�����ֹװ�� */
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW; /* ����������ֵΪ0ʱ���ڼĴ���TBPRDװ��Ӱ�ӼĴ�����ֵ */
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; /* ͬ���ź����ѡ�񣬵�ʱ������������0ʱ��ePWMxSYNCO�ź���� */
	EPwm1Regs.TBCTL.bit.SWFSYNC = 0; /* ���ǿ��ͬ�����壬д0û��Ч�� */
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;

	EPwm1Regs.ETSEL.bit.INTSEL = ET_DCAEVT1SOC; /* ���� */
	EPwm1Regs.ETSEL.bit.INTEN = 0; /* Disable EPWMx_INT generation */
	EPwm1Regs.ETSEL.bit.SOCASEL = 2; /* ��TB�������������ڵ�ʱ�� EPWMxSOCA ���彫���� */
	EPwm1Regs.ETSEL.bit.SOCAEN = 1; /* ʹ��EPWMxSOCA pulse */
	EPwm1Regs.ETSEL.bit.SOCBEN = 0;
	EPwm1Regs.ETSEL.bit.SOCBSEL = 0;

	EPwm1Regs.ETPS.bit.SOCAPRD = 1;
}

