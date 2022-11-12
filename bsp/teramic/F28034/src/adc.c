/******************************************************************
 �� �� ����     adc.c
 D S P��       DSC28034
 ʹ �� �⣺     
 ��     �ã�      
 ˵     ����      �ṩadc.c�ӿڳ�ʼ������
 ---------------------------- ʹ��˵�� ----------------------------
 ����������


 �� ����V1.0.x
 ʱ �䣺2021��11��13��
 �� �ߣ�
 @ mail��support@mail.haawking.com
 ******************************************************************/

#include "adc.h"

/******************************************************************
 ��������void  ADC_Init(void)
 ��	������
 ����ֵ����
 ��	�ã�����ADCINA7ΪADCת��ͨ����6��clock�Ĳ������ڣ�����ԴΪepwm1��ADCSOCA
 ******************************************************************/

void ADC_Init(void)
{
	EALLOW;
	AdcRegs.ADCCTL1.bit.ADCENABLE = 1; /* ʹ��ADC������ADC�ϵ磬д0��Ч��д1ʹ��ADC */

	/* AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1;     ��ϵͳʱ����120M��ʱ��ʹ������Ĵ�����λ������Ϊ1��ΪSYSCLK/4 */
	/* ��ϵͳʱ����60M��ʱ�򣬿��Ա���Ĭ��ֵ0��ΪSYSCLK/2 */
	AdcRegs.ADCCTL2.bit.CLKDIV2EN = 1;

	while(AdcRegs.ADCCTL1.bit.ADCRDY != 1)
	{

	} /* ADC ��λ��ɣ���ADCRDY = 1��ʾ��λ��ɣ�0��ʾ��λδ��� */
	AdcRegs.INTSEL1N2.bit.INT1SEL = 0; /* ADCINT1 EOC Դѡ�� EOC0 ��Ϊ ADCINT1��Ϊ����Դ */
	AdcRegs.INTSEL1N2.bit.INT1E = 1; /*  ADCINT1�ж�ʹ�� */
	AdcRegs.INTSEL1N2.bit.INT1CONT = 0; /* û�н�һ�����ж��������ֱ���ж��źű�־λ����� */
	AdcRegs.SOCPRICTL.bit.ONESHOT = 0; /* One shot ģʽ��ʹ�� */
	AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0;
	AdcRegs.SOCPRICTL.bit.RRPOINTER = 0; /* SOC0�����һ��ת����SOC1����ߵ�round robin���� */
	AdcRegs.ADCSOC0CTL.bit.ACQPS = 0; /* ���ڵĲ���������2n+6��clock cycles */
	AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x7; /* ѡ��ת��������ͨ����ADCINA5 */
	AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x5; /* SOC0�Ĵ���Դ��epwm1��ADCSOCA */
	/* ADCSOCFRC1  ADCINTSOCSEL1 */
	/* 	AdcRegs.ADCINTSOCSEL1.bit.SOC0 = 1; */
	/* 	AdcRegs.ADCSOCFRC1.bit.SOC0 = 1; */
	/* 	AdcRegs.INTSEL1N2.bit.INT1CONT = 1; */
	EDIS;
}

