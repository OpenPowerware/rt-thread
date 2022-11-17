#ifndef _DSP2803x_ADC_H
#define _DSP2803x_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

//
// ADC Individual Register Bit Definition:
// 
struct ADCCTL1_BITS
{                          // bits   description
    Uint32 TEMPCONV:1;     // 0      Temperature Sensor connection
    Uint32 VREFLOCONV:1;   // 1      VSSA Connection
    Uint32 INTPULSEPOS:1;  // 2      INT pulse generation control
    Uint32 ADCREFSEL:1;    // 3  
    Uint32 ADCRDY:1;       // 4      ADC Ready                            new
    Uint32 ADCREFPWD:1;    // 5      
    Uint32 ADCBGPWD:1;     // 6      
    Uint32 ADCPWDN:1;      // 7     
    Uint32 ADCBSYCHN:5;    // 12:8   ADC busy on a channel
    Uint32 ADCBSY:1;       // 13     ADC busy signal
    Uint32 ADCENABLE:1;    // 14     ADC enable
    Uint32 RESET:1;        // 15     ADC Master reset
    Uint32 rsvd2:16;       // 31:16  Reserved    
};

union ADCCTL1_REG
{
    Uint32               all;
    struct  ADCCTL1_BITS  bit;
};


//
///////////////////////////////////////// 
//
struct ADCCTL2_BITS         
{                             // bits   description
    Uint32 CLKDIV2EN:1;       // 0      ADC input clock /2 enable
    Uint32 ADCNONOVERLAP:1;   // 1      ADCNONOVERLAP Control bit
    Uint32 SELDO_HS_LU:1;     // 2      ADC result signed or unsigned        new
    Uint32 EN_ALG_MEAN:1;     // 3      Temprature mean alg enable           new
    Uint32 ALG_MEAN:2;        // 5:4    Mean alg sample points default = 10  new
    Uint32 ADJ_TD_GA:4;       // 9:6    Gain trim,default = 1000             new
    Uint32 ADJ_TD_OS:4;       // 13:10  Offset trim,default = 1000           new
    Uint32 rsvd1:2;           // 15:14  Reserved
    Uint32 rsvd2:16;          // 31:16  Reserved    
};

union ADCCTL2_REG
{
    Uint32               all;
    struct  ADCCTL2_BITS  bit;   
};


//
////////////////////////////////////////
//
struct ADCINT_BITS
{                       // bits   description
	Uint32 ADCINT1:1;   // 0      ADC Interrupt Flag 1
	Uint32 ADCINT2:1;   // 1      ADC Interrupt Flag 2
	Uint32 ADCINT3:1;   // 2      ADC Interrupt Flag 3
	Uint32 ADCINT4:1;   // 3      ADC Interrupt Flag 4
	Uint32 ADCINT5:1;   // 4      ADC Interrupt Flag 5
	Uint32 ADCINT6:1;   // 5      ADC Interrupt Flag 6
	Uint32 ADCINT7:1;   // 6      ADC Interrupt Flag 7
	Uint32 ADCINT8:1;   // 7      ADC Interrupt Flag 8
	Uint32 ADCINT9:1;   // 8      ADC Interrupt Flag 9
	Uint32 rsvd1:7;     // 15:9   reserved
	Uint32 rsvd2:16;    // 31:16  reserved
};

union ADCINT_REG
{
	Uint32                 all;
	struct ADCINT_BITS  bit;
};


//
///////////////////////////////////////
//
struct INTSEL1N2_BITS
{                        // bits   description
	Uint32 INT1SEL:5;    // 4:0    INT1 EOC Source Select
	Uint32 INT1E:1;      // 5      INT1 Interrupt Enable
	Uint32 INT1CONT:1;   // 6      INT1 Continuous Mode Enable
	Uint32 rsvd1:1;      // 7      reserved
	Uint32 INT2SEL:5;    // 12:8   INT2 EOC Source Select
	Uint32 INT2E:1;      // 13     INT2 Interrupt Enable
	Uint32 INT2CONT:1;   // 14     INT2 Continuous Mode Enable
	Uint32 rsvd2:1;      // 15     reserved
	Uint32 rsvd3:16;     // 31:16  reserved
};

union INTSEL1N2_REG
{
	Uint32   all;
	struct INTSEL1N2_BITS  bit;
};


//
//////////////////////////////////////
//
struct INTSEL3N4_BITS
{                       // bits   description
	Uint32 INT3SEL:5;   // 4:0    INT3 EOC Source Select
	Uint32 INT3E:1;     // 5      INT3 Interrupt Enable
	Uint32 INT3CONT:1;  // 6      INT3 Continuous Mode Enable
	Uint32 rsvd1:1;     // 7      reserved
	Uint32 INT4SEL:5;   // 12:8   INT4 EOC Source Select
	Uint32 INT4E:1;     // 13     INT4 Interrupt Enable
	Uint32 INT4CONT:1;  // 14     INT4 Continuous Mode Enable
	Uint32 rsvd2:1;     // 15     reserved
	Uint32 rsvd3:16;    // 31:16  reserved
};

union INTSEL3N4_REG
{
	Uint32                 all;
	struct INTSEL3N4_BITS  bit;
};

//
//////////////////////////////////////
//
struct INTSEL5N6_BITS
{                        // bits   description
	Uint32 INT5SEL:5;    // 4:0    INT5 EOC Source Select
	Uint32 INT5E:1;      // 5      INT5 Interrupt Enable
	Uint32 INT5CONT:1;   // 6      INT5 Continuous Mode Enable
	Uint32 rsvd1:1;      // 7      reserved
	Uint32 INT6SEL:5;    // 12:8   INT6 EOC Source Select
	Uint32 INT6E:1;      // 13     INT6 Interrupt Enable
	Uint32 INT6CONT:1;   // 14     INT6 Continuous Mode Enable
	Uint32 rsvd2:1;      // 15     reserved
	Uint32 rsvd3:16;     // 31:16  reserved
};

union INTSEL5N6_REG
{
	Uint32                 all;
	struct INTSEL5N6_BITS  bit;
};

//
//////////////////////////////////////
//
struct INTSEL7N8_BITS
{                        // bits   description
	Uint32 INT7SEL:5;    // 4:0    INT7 EOC Source Select
	Uint32 INT7E:1;      // 5      INT7 Interrupt Enable
	Uint32 INT7CONT:1;   // 6      INT7 Continuous Mode Enable
	Uint32 rsvd1:1;      // 7      reserved
	Uint32 INT8SEL:5;    // 12:8   INT8 EOC Source Select
	Uint32 INT8E:1;      // 13     INT8 Interrupt Enable
	Uint32 INT8CONT:1;   // 14     INT8 Continuous Mode Enable
	Uint32 rsvd2:1;      // 15     reserved
	Uint32 rsvd3:16;     // 31:16  reserved
};

union INTSEL7N8_REG
{
	Uint32                 all;
	struct INTSEL7N8_BITS  bit;
};

//
//////////////////////////////////////
//
struct INTSEL9N10_BITS
{                        // bits   description
	Uint32 INT9SEL:5;    // 4:0    INT9 EOC Source Select
	Uint32 INT9E:1;      // 5      INT9 Interrupt Enable
	Uint32 INT9CONT:1;   // 6      INT9 Continuous Mode Enable
	Uint32 rsvd1:9;      // 15:7   reserved
	Uint32 rsvd3:16;     // 31:16  reserved
};

union INTSEL9N10_REG
{
	Uint32                 all;
	struct INTSEL9N10_BITS  bit;
};

//
//////////////////////////////////////
//
struct SOCPRICTL_BITS
{                          // bits   description
	Uint32 SOCPRIORITY:5;  // 4:0    Start-of-conversion Priority
	Uint32 RRPOINTER:6;    // 10:5   Round Robin Pointer
	Uint32 rsvd1:1;        // 14:11  reserved
	Uint32 ONESHOT:1;      // 15     ONESHOT Mode
	Uint32 rsvd2:16;       // 31:16  reserved
};

union SOCPRICTL_REG
{
	Uint32             all;
	struct  SOCPRICTL_BITS bit;
};

//
///////////////////////////////////////
//
struct ADCSAMPLEMODE_BITS
{                         // bits   description
	Uint32 SIMULEN0:1;    // 0      Simutaneous Sampling Enable for SOC0/SOC1
	Uint32 SIMULEN2:1;    // 1      Simutaneous Sampling Enable for SOC2/SOC3
	Uint32 SIMULEN4:1;    // 2      Simutaneous Sampling Enable for SOC4/SOC5
	Uint32 SIMULEN6:1;    // 3      Simutaneous Sampling Enable for SOC6/SOC7
	Uint32 SIMULEN8:1;    // 4      Simutaneous Sampling Enable for SOC8/SOC9
	Uint32 SIMULEN10:1;   // 5      Simutaneous Sampling Enable for SOC10/SOC11
	Uint32 SIMULEN12:1;   // 6      Simutaneous Sampling Enable for SOC12/SOC13
	Uint32 SIMULEN14:1;   // 7      Simutaneous Sampling Enable for SOC14/SOC15
	Uint32 rsvd1:8;       // 15:8   reserved
	Uint32 rsvd2:16;      // 31:16  reserved
};

union ADCSAMPLEMODE_REG
{
	Uint32            all;
	struct ADCSAMPLEMODE_BITS bit;
};


//
//////////////////////////////////////
//
struct ADCINTSOCSEL1_BITS
{                         // bits   description
	Uint32 SOC0:2;        // 1:0    SOC0 ADC Interrupt Trigger Select
	Uint32 SOC1:2;        // 3:2    SOC1 ADC Interrupt Trigger Select
	Uint32 SOC2:2;   	  // 5:4    SOC2 ADC Interrupt Trigger Select
	Uint32 SOC3:2;   	  // 7:6    SOC3 ADC Interrupt Trigger Select
	Uint32 SOC4:2; 	      // 9:8    SOC4 ADC Interrupt Trigger Select
	Uint32 SOC5:2;  	  // 11:10  SOC5 ADC Interrupt Trigger Select
	Uint32 SOC6:2; 	      // 13:12  SOC6 ADC Interrupt Trigger Select
	Uint32 SOC7:2; 		  // 15:14  SOC7 ADC Interrupt Trigger Select	
	Uint32 rsvd1:16;      // 31:16  reserved
};

union ADCINTSOCSEL1_REG
{
	Uint32                   all;
	struct  ADCINTSOCSEL1_BITS bit;
};

//
//////////////////////////////////////
//
struct ADCINTSOCSEL2_BITS
{                         // bits   description
	Uint32 SOC8:2;        // 1:0    SOC8 ADC Interrupt Trigger Select
	Uint32 SOC9:2;        // 3:2    SOC9 ADC Interrupt Trigger Select
	Uint32 SOC10:2;   	  // 5:4    SOC10 ADC Interrupt Trigger Select
	Uint32 SOC11:2;   	  // 7:6    SOC11 ADC Interrupt Trigger Select
	Uint32 SOC12:2; 	  // 9:8    SOC12 ADC Interrupt Trigger Select
	Uint32 SOC13:2;  	  // 11:10  SOC13 ADC Interrupt Trigger Select
	Uint32 SOC14:2; 	  // 13:12  SOC14 ADC Interrupt Trigger Select
	Uint32 SOC15:2; 	  // 15:14  SOC15 ADC Interrupt Trigger Select	
	Uint32 rsvd1:16;      // 31:16  reserved	
};

union ADCINTSOCSEL2_REG
{
	Uint32                   all;
	struct  ADCINTSOCSEL2_BITS bit;
};

//
////////////////////////////////////
//
struct ADCSOC_BITS
{                    // bits   description
	Uint32 SOC0:1;   // 0      Start-of-conversion for CONV0
	Uint32 SOC1:1;   // 1      Start-of-conversion for CONV1
	Uint32 SOC2:1;   // 2      Start-of-conversion for CONV2
	Uint32 SOC3:1;   // 3      Start-of-conversion for CONV3
	Uint32 SOC4:1;   // 4      Start-of-conversion for CONV4
	Uint32 SOC5:1;   // 5      Start-of-conversion for CONV5
	Uint32 SOC6:1;   // 6      Start-of-conversion for CONV6
	Uint32 SOC7:1;   // 7      Start-of-conversion for CONV7
	Uint32 SOC8:1;   // 8      Start-of-conversion for CONV8
	Uint32 SOC9:1;   // 9      Start-of-conversion for CONV9
	Uint32 SOC10:1;  // 10     Start-of-conversion for CONV10
	Uint32 SOC11:1;  // 11     Start-of-conversion for CONV11
	Uint32 SOC12:1;  // 12     Start-of-conversion for CONV12
	Uint32 SOC13:1;  // 13     Start-of-conversion for CONV13
	Uint32 SOC14:1;  // 14     Start-of-conversion for CONV14
	Uint32 SOC15:1;  // 15     Start-of-conversion for CONV15
	Uint32 rsvd1:16; //31:16   reserved
};

union ADCSOC_REG
{
	Uint32                   all;
	struct ADCSOC_BITS       bit;
};

//
////////////////////////////////////
//
struct ADCSOCOVFCLR1_BITS
{
	Uint32 SOC0:1;   //0   Start-of-conversion for CONV0
	Uint32 SOC1:1;   //1   Start-of-conversion for CONV1
	Uint32 SOC2:1;   //2   Start-of-conversion for CONV2
	Uint32 SOC3:1;   //3   Start-of-conversion for CONV3
	Uint32 SOC4:1;   //4   Start-of-conversion for CONV4
	Uint32 SOC5:1;   //5   Start-of-conversion for CONV5
	Uint32 SOC6:1;   //6   Start-of-conversion for CONV6
	Uint32 SOC7:1;   //7   Start-of-conversion for CONV7
	Uint32 SOC8:1;   //8   Start-of-conversion for CONV8
	Uint32 SOC9:1;   //9   Start-of-conversion for CONV9
	Uint32 SOC10:1;  //10  Start-of-conversion for CONV10
	Uint32 SOC11:1;  //11  Start-of-conversion for CONV11
	Uint32 SOC12:1;  //12  Start-of-conversion for CONV12
	Uint32 SOC13:1;  //13  Start-of-conversion for CONV13
	Uint32 SOC14:1;  //14  Start-of-conversion for CONV14
	Uint32 SOC15:1;  //15  Start-of-conversion for CONV15
	Uint32 rsvd1:16; //31:16     reserved
};

union ADCSOCOVFCLR1_REG
{
     Uint32                   all;
     struct ADCSOCOVFCLR1_BITS       bit;
};
//
///////////////////////////////////////
//
struct ADCSOCxCTL_BITS
{                       // bits   description
    Uint32 ACQPS:5;     // 4:0    Acquisition Pulse Size
    Uint32 rsvd3:1;     // 5  	  reserved
    Uint32 CHSEL:4;     // 9:6    SOCx Channel Select
    Uint32 rsvd1:1;     // 10     reserved
    Uint32 TRIGSEL:5;   // 15:11  SOCx Trigger Select
    Uint32 rsvd2:16;    // 31:16  reserved
};

union ADCSOCxCTL_REG
{
	Uint32               all;
	struct ADCSOCxCTL_BITS   bit;
};

//
//////////////////////////////////////
//
struct ADCREFTRIM_BITS
{                               // bits   description
    Uint32 BG_FINE_TRIM:5;      // 4:0    Coarse trim for internal BG
    Uint32 BG_CAORSE_TRIM:4;    // 8:5    Fine trim for internal BG
    Uint32 EXTREF_FINE_TRIM:5;  // 13:9   Fine trim for external reference
    Uint32 rsvd1:18;            // 31:14  reserved
};

union ADCREFTRIM_REG
{
     Uint32             all;
     struct ADCREFTRIM_BITS bit;
};

//
/////////////////////////////////////
//
struct ADCOFFTRIM_BITS
{                         // bits   description
    Uint32 rsvd1:9;       // 8:0    reserved
    Uint32 OFFTRIM:9;     // 17:9    Offset Trim
    Uint32 rsvd2:14;      // 31:18   reserved
};

union ADCOFFTRIM_REG
{
     Uint32                            all;
     struct ADCOFFTRIM_BITS bit;
};

//
////////////////////////////////////
//
struct COMPHYSTCTL_BITS
{                                  // bits   description
     Uint32 rsvd1:1;                // 0      reserved
     Uint32 COMP1_HYST_DISABLE:1;   // 1      Comparator 1 Hysteresis Disable
     Uint32 COMP1_HYST_SEL:2;       // 3:2    cmp1_hyst_sel                      new
     Uint32 rsvd2:2;                // 5:4    reserved
     Uint32 COMP2_HYST_DISABLE:1;   // 6      Comparator 2 Hysteresis Disable
     Uint32 COMP2_HYST_SEL:2;       // 8:7    cmp2_hyst_sel                      new
     Uint32 rsvd3:2;                // 10:9   reserved                           
     Uint32 COMP3_HYST_DISABLE:1;   // 11     Comparator 3 Hysteresis Disable    28034
     Uint32 COMP3_HYST_SEL:2;       // 13:12  cmp3_hyst_sel                      28034
     Uint32 rsvd4:18;               // 31:14  reserved     
};

union COMPHYSTCTL_REG
{
     Uint32                   all;
     struct COMPHYSTCTL_BITS bit;
};

//
///////////////////////////////////////
//
struct ADCREV_BITS
{
      Uint32 TYPE_T:8;      //7:0   TYPE
	  Uint32 REV:8;         //15:8   version
      Uint32 rsvd1:16;      //31:16    reserved     
};

union ADCREV_REG
{
      Uint32                   all;
      struct ADCREV_BITS bit;
};


//
///////////////////////////////////////
//
struct ADC_REGS
{
	union ADCCTL1_REG        ADCCTL1;           //ADC Control 1
	union ADCCTL2_REG        ADCCTL2;           //ADC Control 2
	union ADCINT_REG         ADCINTFLG;         //ADC Interrupt Flag 
	union ADCINT_REG         ADCINTFLGCLR;      //ADC Interrupt Flag Clear
	union ADCINT_REG         ADCINTOVF;         //ADC Interrupt Overflow

	union ADCINT_REG         ADCINTOVFCLR;      //ADC Interrupt Overflow Clear

	union INTSEL1N2_REG      INTSEL1N2;         //ADC Interrupt 1 and 2 Selection
	union INTSEL3N4_REG      INTSEL3N4;         //ADC Interrupt 3 and 4 Selection
	union INTSEL5N6_REG      INTSEL5N6;         //ADC Interrupt 5 and 6 Selection
	union INTSEL7N8_REG      INTSEL7N8;         //ADC Interrupt 7 and 8 Selection
	union INTSEL9N10_REG     INTSEL9N10;        //ADC Interrupt 9 and 10 Selection


	union SOCPRICTL_REG      SOCPRICTL;         //ADC SOC Priority control


	union ADCSAMPLEMODE_REG  ADCSAMPLEMODE;     //ADC Sampling Mode


	union ADCINTSOCSEL1_REG  ADCINTSOCSEL1;     //ADC Interrupt SOC Selection 1
	union ADCINTSOCSEL2_REG  ADCINTSOCSEL2;     //ADC Interrupt SOC Selection 2


	union ADCSOC_REG         ADCSOCFLG1;        //ADC SOC Flag 1


	union ADCSOC_REG         ADCSOCFRC1;        //ADC SOC Flag Force 1


	union ADCSOC_REG         ADCSOCOVF1;        //ADC SOC Overflow 1


	union ADCSOC_REG         ADCSOCOVFCLR1;     //ADC SOC Overflow Clear 1


	union ADCSOCxCTL_REG     ADCSOC0CTL;        //ADC SOC0 Control
	union ADCSOCxCTL_REG     ADCSOC1CTL;        //ADC SOC1 Control
	union ADCSOCxCTL_REG     ADCSOC2CTL;        //ADC SOC2 Control
	union ADCSOCxCTL_REG     ADCSOC3CTL;        //ADC SOC3 Control
	union ADCSOCxCTL_REG     ADCSOC4CTL;        //ADC SOC4 Control
	union ADCSOCxCTL_REG     ADCSOC5CTL;        //ADC SOC5 Control
	union ADCSOCxCTL_REG     ADCSOC6CTL;        //ADC SOC6 Control
	union ADCSOCxCTL_REG     ADCSOC7CTL;        //ADC SOC7 Control
	union ADCSOCxCTL_REG     ADCSOC8CTL;        //ADC SOC8 Control
	union ADCSOCxCTL_REG     ADCSOC9CTL;        //ADC SOC9 Control
	union ADCSOCxCTL_REG     ADCSOC10CTL;       //ADC SOC10 Control
	union ADCSOCxCTL_REG     ADCSOC11CTL;       //ADC SOC11 Control
	union ADCSOCxCTL_REG     ADCSOC12CTL;       //ADC SOC12 Control
	union ADCSOCxCTL_REG     ADCSOC13CTL;       //ADC SOC13 Control
	union ADCSOCxCTL_REG     ADCSOC14CTL;       //ADC SOC14 Control
	union ADCSOCxCTL_REG     ADCSOC15CTL;       //ADC SOC15 Control


	union ADCREFTRIM_REG     ADCREFTRIM;        //Reference Trim Register
	union ADCOFFTRIM_REG     ADCOFFTRIM;        //Offset Trim Register

	
	union COMPHYSTCTL_REG    COMPHYSTCTL;       //COMP Hysteresis Control Register


	union ADCREV_REG         ADCREV;            

};

struct ADC_RESULT_REGS
{

	Uint32 ADCRESULT0;    // Conversion Result Buffer 0
	Uint32 ADCRESULT1;    // Conversion Result Buffer 1
	Uint32 ADCRESULT2;    // Conversion Result Buffer 2
	Uint32 ADCRESULT3;    // Conversion Result Buffer 3
	Uint32 ADCRESULT4;    // Conversion Result Buffer 4
	Uint32 ADCRESULT5;    // Conversion Result Buffer 5
	Uint32 ADCRESULT6;    // Conversion Result Buffer 6
	Uint32 ADCRESULT7;    // Conversion Result Buffer 7
	Uint32 ADCRESULT8;    // Conversion Result Buffer 8
	Uint32 ADCRESULT9;    // Conversion Result Buffer 9
	Uint32 ADCRESULT10;   // Conversion Result Buffer 10
	Uint32 ADCRESULT11;   // Conversion Result Buffer 11
	Uint32 ADCRESULT12;   // Conversion Result Buffer 12
	Uint32 ADCRESULT13;   // Conversion Result Buffer 13
	Uint32 ADCRESULT14;   // Conversion Result Buffer 14
	Uint32 ADCRESULT15;   // Conversion Result Buffer 15
};


extern volatile struct ADC_REGS *const P_AdcRegs;
extern volatile struct ADC_RESULT_REGS *const P_AdcResult;

extern volatile struct ADC_REGS AdcRegs;
extern volatile struct ADC_RESULT_REGS AdcResult;

#ifdef __cplusplus

} 

#endif


#endif