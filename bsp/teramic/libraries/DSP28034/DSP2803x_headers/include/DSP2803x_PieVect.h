

#ifndef DSP2803x_PIE_VECT_H
#define DSP2803x_PIE_VECT_H

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------
// PIE Interrupt Vector Table Definition:
//


// Define Vector Table:
struct PIE_VECT_TABLE {

// Group 1 PIE Peripheral Vectors:   //200
      PINT     ADCINT1;   // ADC - if Group 10 ADCINT1 is enabled, this must be rsvd1_1
      PINT     ADCINT2;   // ADC - if Group 10 ADCINT2 is enabled, this must be rsvd1_2
      PINT     rsvd1_3;
      PINT     XINT1;
      PINT     XINT2;
      PINT     ADCINT9;   // ADC
      PINT     TINT0;     // Timer 0
      PINT     WAKEINT;   // WD

// Group 2 PIE Peripheral Vectors: //220
      PINT     EPWM1_TZINT; // EPWM-1
      PINT     EPWM2_TZINT; // EPWM-2
      PINT     EPWM3_TZINT; // EPWM-3
      PINT     EPWM4_TZINT; // EPWM-4
      PINT     EPWM5_TZINT;
      PINT     EPWM6_TZINT;
      PINT     EPWM7_TZINT;
      PINT     rsvd2_8;

// Group 3 PIE Peripheral Vectors:// 240
      PINT     EPWM1_INT;  // EPWM-1
      PINT     EPWM2_INT;  // EPWM-2
      PINT     EPWM3_INT;  // EPWM-3
      PINT     EPWM4_INT;  // EPWM-4
      PINT     EPWM5_INT;
      PINT     EPWM6_INT;
      PINT     EPWM7_INT;
      PINT     rsvd3_8;

// Group 4 PIE Peripheral Vectors: //260
      PINT     ECAP1_INT; // ECAP-1
      PINT     rsvd4_2;
      PINT     rsvd4_3;
      PINT     rsvd4_4;
      PINT     rsvd4_5;
      PINT     rsvd4_6;
      PINT     HRCAP1_INT;
      PINT     HRCAP2_INT;

// Group 5 PIE Peripheral Vectors: //280

      PINT     EQEP1_INT;
      PINT     EQEP2_INT;
      PINT     rsvd5_3;
      PINT     rsvd5_4;
      PINT     rsvd5_5;
      PINT     rsvd5_6;
      PINT     rsvd5_7;
      PINT     rsvd5_8;

// Group 6 PIE Peripheral Vectors://2A0
      PINT     SPIRXINTA; // SPI-A
      PINT     SPITXINTA; // SPI-A
      PINT     SPIRXINTB; // SPI-A
      PINT     SPITXINTB; // SPI-A
      PINT     rsvd6_5;
      PINT     rsvd6_6;
      PINT     rsvd6_7;
      PINT     rsvd6_8;

// Group 7 PIE Peripheral Vectors://2C0
      PINT     DINTCH1;
      PINT     DINTCH2;
      PINT     DINTCH3;
      PINT     DINTCH4;
      PINT     DINTCH5;
      PINT     DINTCH6;
      PINT     rsvd7_7;
      PINT     rsvd7_8;

// Group 8 PIE Peripheral Vectors://2E0
      PINT     I2CINT1A;  // I2C-A
      PINT     I2CINT2A;  // 
      PINT     rsvd8_3;
      PINT     rsvd8_4;
      PINT     rsvd8_5;
      PINT     rsvd8_6;
      PINT     rsvd8_7;
      PINT     rsvd8_8;

// Group 9 PIE Peripheral Vectors://300

      PINT     SCIRXINTA;  // SCI-A
      PINT     SCITXINTA;
      PINT     LIN0INTA;   // LIN-A
      PINT     LIN1INTA;   // LIN-A
      PINT     ECAN0INTA;
      PINT     ECAN1INTA;
      PINT     rsvd9_7;
      PINT     rsvd9_8;

// Group 10 PIE Peripheral Vectors://320
      PINT     rsvd10_1; // ADC
      PINT     rsvd10_2; // ADC
      PINT     ADCINT3;  // ADC
      PINT     ADCINT4;  // ADC
      PINT     ADCINT5;  // ADC
      PINT     ADCINT6;  // ADC
      PINT     ADCINT7;  // ADC
      PINT     ADCINT8;  // ADC
// Group 11 PIE Peripheral Vectors://340
      PINT     rsvd11_1;
      PINT     rsvd11_2;
      PINT     rsvd11_3;
      PINT     rsvd11_4;
      PINT     rsvd11_5;
      PINT     rsvd11_6;
      PINT     rsvd11_7;
      PINT     rsvd11_8;
// Group 12 PIE Peripheral Vectors://360
      PINT     XINT3;
      PINT     rsvd12_2;
      PINT     rsvd12_3;
      PINT     rsvd12_4;
      PINT     rsvd12_5;
      PINT     rsvd12_6;
      PINT     rsvd12_7;
      PINT     rsvd12_8;
// Non-Peripheral Interrupts:  
      PINT	   TINT1;	  // CPU-Timer1  //380
      PINT     rsvd13_2;
      PINT     rsvd13_3;
      PINT     rsvd13_4;
      PINT     rsvd13_5;
      PINT     rsvd13_6;
      PINT     rsvd13_7;
      PINT     rsvd13_8;	
//	  
      PINT	   TINT2;	  // CPU-Timer2  //3A0
      PINT     rsvd14_2;
      PINT     rsvd14_3;
      PINT     rsvd14_4;
      PINT     rsvd14_5;
      PINT     rsvd14_6;
      PINT     rsvd14_7;
      PINT     rsvd14_8;

	  
};

//---------------------------------------------------------------------------
// PIE Interrupt Vector Table External References & Function Declarations:
//
//extern struct PIE_VECT_TABLE PieVectTable;

extern  volatile struct PIE_VECT_TABLE PieVectTable;
extern  volatile struct PIE_VECT_TABLE *const P_PieVectTable;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif    
//===========================================================================
// End of file.
//===========================================================================

