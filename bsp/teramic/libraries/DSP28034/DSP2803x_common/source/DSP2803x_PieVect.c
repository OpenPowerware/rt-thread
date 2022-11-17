
#include "DSP2803x_Device.h"     //  Headerfile Include File
#include "DSP2803x_Examples.h"  




const struct PIE_VECT_TABLE PieVectTableInit = {

// Group 1 PIE Vectors
      ADCINT1_ISR,     // 1.1 ADC  // if this is rsvd_ISR, then INT10.1 should be defined as ADCINT1_ISR
      ADCINT2_ISR,     // 1.2 ADC  // if this is rsvd_ISR, then INT10.2 should be defined as ADCINT2_ISR
      rsvd_ISR,        // 1.3
      XINT1_ISR,       // 1.4 External Interrupt
      XINT2_ISR,       // 1.5 External Interrupt
      ADCINT9_ISR,     // 1.6 ADC
      TINT0_ISR,       // 1.7 Timer 0
      WAKEINT_ISR,     // 1.8 WD, Low Power

// Group 2 PIE Vectors
      EPWM1_TZINT_ISR, // 2.1 EPWM-1 Trip Zone
      EPWM2_TZINT_ISR, // 2.2 EPWM-2 Trip Zone
      EPWM3_TZINT_ISR, // 2.3 EPWM-3 Trip Zone
      EPWM4_TZINT_ISR, // 2.4 EPWM-4 Trip Zone
      rsvd_ISR,        // 2.5
      rsvd_ISR,        // 2.6
      rsvd_ISR,        // 2.7
      rsvd_ISR,        // 2.8

// Group 3 PIE Vectors
      EPWM1_INT_ISR,   // 3.1 EPWM-1 Interrupt
      EPWM2_INT_ISR,   // 3.2 EPWM-2 Interrupt
      EPWM3_INT_ISR,   // 3.3 EPWM-3 Interrupt
      EPWM4_INT_ISR,   // 3.4 EPWM-4 Interrupt
      rsvd_ISR,        // 3.5
      rsvd_ISR,        // 3.6
      rsvd_ISR,        // 3.7
      rsvd_ISR,        // 3.8

// Group 4 PIE Vectors
      ECAP1_INT_ISR,   // 4.1 ECAP-1
      ECAP1_INT_ISR,        // 4.2
      rsvd_ISR,        // 4.3
      rsvd_ISR,        // 4.4
      rsvd_ISR,        // 4.5
      rsvd_ISR,        // 4.6
      rsvd_ISR,        // 4.7
      rsvd_ISR,        // 4.8

// Group 5 PIE Vectors
      EQEP1_INT_ISR,        // 5.1
      rsvd_ISR,        // 5.2
      rsvd_ISR,        // 5.3
      rsvd_ISR,        // 5.4
      rsvd_ISR,        // 5.5
      rsvd_ISR,        // 5.6
      rsvd_ISR,        // 5.7
      rsvd_ISR,        // 5.8

// Group 6 PIE Vectors
      SPIRXINTA_ISR,   // 6.1 SPI-A
      SPITXINTA_ISR,   // 6.2 SPI-A
      rsvd_ISR,        // 6.3
      rsvd_ISR,        // 6.4
      rsvd_ISR,        // 6.5
      rsvd_ISR,        // 6.6
      rsvd_ISR,        // 6.7
      rsvd_ISR,        // 6.8

// Group 7 PIE Vectors
      DINTCH1_ISR,        // 7.1
      DINTCH2_ISR,        // 7.2
      DINTCH3_ISR,        // 7.3
      DINTCH4_ISR,        // 7.4
	  DINTCH5_ISR,        // 7.5
	  DINTCH6_ISR,        // 7.6
      rsvd_ISR,        // 7.7
      rsvd_ISR,        // 7.8

// Group 8 PIE Vectors
      I2CINT1A_ISR,    // 8.1 I2C
      rsvd_ISR,    // 8.2 I2C
      rsvd_ISR,        // 8.3
      rsvd_ISR,        // 8.4
      rsvd_ISR,        // 8.5
      rsvd_ISR,        // 8.6
      rsvd_ISR,        // 8.7
      rsvd_ISR,        // 8.8

// Group 9 PIE Vectors
      SCITXINTA_ISR,   // 9.1 SCI-A
      SCIRXINTA_ISR,   // 9.2 SCI-A
      LINA_INT0_ISR,        // 9.3
      LINA_INT1_ISR,        // 9.4
      ECANA_INT0_ISR,        // 9.5
      ECANA_INT1_ISR,        // 9.6
      rsvd_ISR,        // 9.7
      rsvd_ISR,        // 9.8

// Group 10 PIE Vectors
      rsvd_ISR,        // 10.1 If this is ADCINT1_ISR, then INT1.1 should be defined as rsvd_ISR
      rsvd_ISR,        // 10.2 If this is ADCINT2_ISR, then INT1.2 should be defined as rsvd_ISR
      ADCINT3_ISR,     // 10.3 ADC
      ADCINT4_ISR,     // 10.4 ADC
      ADCINT5_ISR,     // 10.5 ADC
      ADCINT6_ISR,     // 10.6 ADC
      ADCINT7_ISR,     // 10.7 ADC
      ADCINT8_ISR,     // 10.8 ADC

// Group 11 PIE Vectors
      rsvd_ISR,        // 11.1
      rsvd_ISR,        // 11.2
      rsvd_ISR,        // 11.3
      rsvd_ISR,        // 11.4
      rsvd_ISR,        // 11.5
      rsvd_ISR,        // 11.6
      rsvd_ISR,        // 11.7
      rsvd_ISR,        // 11.8

// Group 12 PIE Vectors
      XINT3_ISR,       // 12.1 External Interrupt
      rsvd_ISR,        // 12.2
      rsvd_ISR,        // 12.3
      rsvd_ISR,        // 12.4
      rsvd_ISR,        // 12.5
      rsvd_ISR,        // 12.6
      rsvd_ISR,        // 12.7
      rsvd_ISR,        // 12.8
	// Non-Peripheral Interrupts:  
	  TINT1_ISR,	  // CPU-Timer1  //380
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,	
	//	  
	  TINT2_ISR,	  // CPU-Timer2  //3A0
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,
	  rsvd_ISR,
      rsvd_ISR,

};

//---------------------------------------------------------------------------
// InitPieVectTable:
//---------------------------------------------------------------------------
// This function initializes the PIE vector table to a known state.
// This function must be executed after boot time.
//

void InitPieVectTable(void)
{
	int16	i;
	Uint32 *Source = (void *) &PieVectTableInit;
	Uint32 *Dest = (void *) &PieVectTable;

	// Do not write over first 3 32-bit locations (these locations are
	// initialized by Boot ROM with boot variables)

//	Source = Source + 3;
//	Dest = Dest + 3;

	EALLOW;
	for(i=0; i < 96; i++)
		*Dest++ = *Source++;
	EDIS;
	
	// Enable the PIE Vector Table
//	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
	EINT;
}

//===========================================================================
// End of file.
//===========================================================================
