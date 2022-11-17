
#ifndef DSP2803x_DEFAULT_ISR_H
#define DSP2803x_DEFAULT_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------
// Default Interrupt Service Routine Declarations:
//
// The following function prototypes are for the
// default ISR routines used with the default PIE vector table.
// This default vector table is found in the DSC28027_PieVect.h
// file.
//

// For compatibility with header files V1.00 and earlier.
//#define TINT1_ISR INT13_ISR
//#define TINT2_ISR INT14_ISR

// Non-Peripheral Interrupts:
 void INTERRUPT INT13_ISR(void);     // INT13 or CPU-Timer 1
 void INTERRUPT INT14_ISR(void);     // INT14 or CPU-Timer 2
 void INTERRUPT DATALOG_ISR(void);   // Datalogging interrupt
 void INTERRUPT RTOSINT_ISR(void);   // RTOS interrupt
 void INTERRUPT EMUINT_ISR(void);    // Emulation interrupt
 void INTERRUPT NMI_ISR(void);       // Non-maskable interrupt
 void INTERRUPT ILLEGAL_ISR(void);   // Illegal operation TRAP
 void INTERRUPT USER1_ISR(void);     // User Defined trap 1
 void INTERRUPT USER2_ISR(void);     // User Defined trap 2
 void INTERRUPT USER3_ISR(void);     // User Defined trap 3
 void INTERRUPT USER4_ISR(void);     // User Defined trap 4
 void INTERRUPT USER5_ISR(void);     // User Defined trap 5
 void INTERRUPT USER6_ISR(void);     // User Defined trap 6
 void INTERRUPT USER7_ISR(void);     // User Defined trap 7
 void INTERRUPT USER8_ISR(void);     // User Defined trap 8
 void INTERRUPT USER9_ISR(void);     // User Defined trap 9
 void INTERRUPT USER10_ISR(void);    // User Defined trap 10
 void INTERRUPT USER11_ISR(void);    // User Defined trap 11
 void INTERRUPT USER12_ISR(void);    // User Defined trap 12

// Group 1 PIE Interrupt Service Routines:
 void  INTERRUPT ADCINT1_ISR(void);   // ADC INT1 ISR - 1.1 OR 10.1
 void  INTERRUPT ADCINT2_ISR(void);   // ADC INT2 ISR - 1.2 OR 10.2
 void  INTERRUPT XINT1_ISR(void);     // External interrupt 1
 void  INTERRUPT XINT2_ISR(void);     // External interrupt 2
 void  INTERRUPT ADCINT9_ISR(void);   // ADC INT9 ISR
 void  INTERRUPT TINT0_ISR(void);     // Timer 0
 void  INTERRUPT WAKEINT_ISR(void);   // WD

// Group 2 PIE Interrupt Service Routines:
 void INTERRUPT EPWM1_TZINT_ISR(void);    // EPWM-1
 void INTERRUPT EPWM2_TZINT_ISR(void);    // EPWM-2
 void INTERRUPT EPWM3_TZINT_ISR(void);    // EPWM-3
 void INTERRUPT EPWM4_TZINT_ISR(void);    // EPWM-4

// Group 3 PIE Interrupt Service Routines:
 void INTERRUPT EPWM1_INT_ISR(void);    // EPWM-1
 void INTERRUPT EPWM2_INT_ISR(void);    // EPWM-2
 void INTERRUPT EPWM3_INT_ISR(void);    // EPWM-3
 void INTERRUPT EPWM4_INT_ISR(void);    // EPWM-4

// Group 4 PIE Interrupt Service Routines:
 void INTERRUPT ECAP1_INT_ISR(void);    // ECAP-1

 void INTERRUPT EQEP1_INT_ISR(void);

// Group 5 PIE Interrupt Service Routines:

// Group 6 PIE Interrupt Service Routines:
 void INTERRUPT SPIRXINTA_ISR(void);   // SPI-A
 void INTERRUPT SPITXINTA_ISR(void);   // SPI-A

// Group 7 PIE Interrupt Service Routines:
 void INTERRUPT DINTCH1_ISR(void);   // DMA Channel 1
 void INTERRUPT DINTCH2_ISR(void);   // DMA Channel 2
 void INTERRUPT DINTCH3_ISR(void);   // DMA Channel 3
 void INTERRUPT DINTCH4_ISR(void);   // DMA Channel 4
 void INTERRUPT DINTCH5_ISR(void);   // DMA Channel 4
 void INTERRUPT DINTCH6_ISR(void);   // DMA Channel 4

// Group 8 PIE Interrupt Service Routines:
 void INTERRUPT I2CINT1A_ISR(void);     // I2C-A
 void INTERRUPT I2CINT2A_ISR(void);     // I2C-A

// Group 9 PIE Interrupt Service Routines:
 void INTERRUPT SCITXINTA_ISR(void); // SCI-A
 void INTERRUPT SCIRXINTA_ISR(void); // SCI-A

 void INTERRUPT LINA_INT0_ISR(void);//LIN-A
 void INTERRUPT LINA_INT1_ISR(void);//LIN-A

 void INTERRUPT ECANA_INT0_ISR(void);//ECAN-A
 void INTERRUPT ECANA_INT1_ISR(void);//ECAN-A

// Group 10 PIE Interrupt Service Routines:
// ADC INT1 ISR - 1.1 or 10.1
// ADC INT2 ISR - 1.2 or 10.2
 void INTERRUPT ADCINT3_ISR(void);  // ADC INT3 ISR
 void INTERRUPT ADCINT4_ISR(void);  // ADC INT4 ISR
 void INTERRUPT ADCINT5_ISR(void);  // ADC INT5 ISR
 void INTERRUPT ADCINT6_ISR(void);  // ADC INT6 ISR
 void INTERRUPT ADCINT7_ISR(void);  // ADC INT7 ISR
 void INTERRUPT ADCINT8_ISR(void);  // ADC INT8 ISR

// Group 11 PIE Interrupt Service Routines:

// Group 12 PIE Interrupt Service Routines:
 void INTERRUPT XINT3_ISR (void); // External interrupt 3

// Catch-all for Reserved Locations For testing purposes:
 void INTERRUPT PIE_RESERVED(void);       // Reserved for test
 void INTERRUPT rsvd_ISR(void);           // for test
 void INTERRUPT INT_NOTUSED_ISR(void);    // for unused interrupts
void INTERRUPT TINT1_ISR(void);
void INTERRUPT TINT2_ISR(void); 

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif   

//===========================================================================
// End of file.
//===========================================================================
