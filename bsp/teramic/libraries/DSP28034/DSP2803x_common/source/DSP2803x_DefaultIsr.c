
#include "DSP2803x_Device.h"     //  Headerfile Include File
#include "DSP2803x_Examples.h"   //  Examples Include File

// Connected to INT13 of CPU (use MINT13 mask):
 void INTERRUPT INT13_ISR(void)     // INT13 or CPU-Timer1
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 // asm ("      ESTOP0");
  for(;;);
}

 void INTERRUPT INT14_ISR(void)     // INT14 or CPU-Timer2
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 // asm ("      ESTOP0");
  for(;;);
}

 void INTERRUPT DATALOG_ISR(void)   // Datalogging interrupt
{
   // Insert ISR Code here

   // Next two lines for debug only to halt the processor here
   // Remove after inserting ISR Code
  // asm ("      ESTOP0");
   for(;;);
}

 void INTERRUPT RTOSINT_ISR(void)   // RTOS interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 // asm ("      ESTOP0");
  for(;;);
}

 void INTERRUPT EMUINT_ISR(void)    // Emulation interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 // asm ("      ESTOP0");
  for(;;);
}

 void INTERRUPT NMI_ISR(void)       // Non-maskable interrupt
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT ILLEGAL_ISR(void)   // Illegal operation TRAP
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm("          ESTOP0");
  for(;;);

}

void  INTERRUPT USER1_ISR(void)     // User Defined trap 1
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);

}

void INTERRUPT USER2_ISR(void)     // User Defined trap 2
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 // asm ("      ESTOP0");
  for(;;);

}

void INTERRUPT USER3_ISR(void)     // User Defined trap 3
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 // asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT USER4_ISR(void)     // User Defined trap 4
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT USER5_ISR(void)     // User Defined trap 5
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT USER6_ISR(void)     // User Defined trap 6
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT USER7_ISR(void)     // User Defined trap 7
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT USER8_ISR(void)     // User Defined trap 8
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void  INTERRUPT USER9_ISR(void)     // User Defined trap 9
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT USER10_ISR(void)    // User Defined trap 10
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT USER11_ISR(void)    // User Defined trap 11
{
  // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT USER12_ISR(void)     // User Defined trap 12
{
 // Insert ISR Code here

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);

}

// -----------------------------------------------------------
// PIE Group 1 - MUXed into CPU INT1
// -----------------------------------------------------------
// INT1.1
void INTERRUPT ADCINT1_ISR(void)   // ADC  (Can also be ISR for INT10.1 when enabled)
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code

  //asm ("      ESTOP0");
  for(;;);

}

// INT1.2
void INTERRUPT ADCINT2_ISR(void)  // ADC  (Can also be ISR for INT10.2 when enabled)
{

  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code

  //asm("	  ESTOP0");
  for(;;);

}


// INT1.3 - Reserved

// INT1.4
void INTERRUPT  XINT1_ISR(void)
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);

}

// INT1.5
void INTERRUPT  XINT2_ISR(void)
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);

}

// INT1.6
void INTERRUPT  ADCINT9_ISR(void)     // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// INT1.7
void INTERRUPT  TINT0_ISR(void)      // CPU-Timer 0
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// INT1.8
void INTERRUPT  WAKEINT_ISR(void)    // WD, LOW Power
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// -----------------------------------------------------------
// PIE Group 2 - MUXed into CPU INT2
// -----------------------------------------------------------

// INT2.1
void INTERRUPT EPWM1_TZINT_ISR(void)    // EPWM-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// INT2.2
void INTERRUPT EPWM2_TZINT_ISR(void)    // EPWM-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
 // asm ("      ESTOP0");
  for(;;);
}

// INT2.3
void INTERRUPT EPWM3_TZINT_ISR(void)    // EPWM-3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// INT2.4
void INTERRUPT EPWM4_TZINT_ISR(void)    // EPWM-4
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// INT2.5 - Reserved
// INT2.6 - Reserved
// INT2.7 - Reserved
// INT2.8 - Reserved

// -----------------------------------------------------------
// PIE Group 3 - MUXed into CPU INT3
// -----------------------------------------------------------

// INT 3.1
void INTERRUPT EPWM1_INT_ISR(void)     // EPWM-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// INT3.2
void INTERRUPT EPWM2_INT_ISR(void)     // EPWM-2
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// INT3.3
void INTERRUPT EPWM3_INT_ISR(void)    // EPWM-3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
  //asm ("      ESTOP0");
  for(;;);
}

// INT3.4
void INTERRUPT EPWM4_INT_ISR(void)    // EPWM-4
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}

// INT3.5 - Reserved
// INT3.6 - Reserved
// INT3.7 - Reserved
// INT3.8 - Reserved

// -----------------------------------------------------------
// PIE Group 4 - MUXed into CPU INT4
// -----------------------------------------------------------

// INT 4.1
void INTERRUPT ECAP1_INT_ISR(void)    // ECAP-1
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}


void INTERRUPT EQEP1_INT_ISR(void)
{
 for(;;);	
}

// INT4.2 - Reserved
// INT4.3 - Reserved
// INT4.4 - Reserved
// INT4.5 - Reserved
// INT4.6 - Reserved
// INT4.7 - Reserved
// INT4.8 - Reserved

// -----------------------------------------------------------
// PIE Group 5 - MUXed into CPU INT5
// -----------------------------------------------------------

// INT5.1 - Reserved
// INT5.2 - Reserved
// INT5.3 - Reserved
// INT5.4 - Reserved
// INT5.5 - Reserved
// INT5.6 - Reserved
// INT5.7 - Reserved
// INT5.8 - Reserved

// -----------------------------------------------------------
// PIE Group 6 - MUXed into CPU INT6
// -----------------------------------------------------------

// INT6.1
void INTERRUPT SPIRXINTA_ISR(void)    // SPI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}

// INT6.2
void INTERRUPT SPITXINTA_ISR(void)     // SPI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}

// INT6.3 - Reserved
// INT6.4 - Reserved
// INT6.5 - Reserved
// INT6.6 - Reserved
// INT6.7 - Reserved
// INT6.8 - Reserved

// -----------------------------------------------------------
// PIE Group 7 - MUXed into CPU INT7
// -----------------------------------------------------------
// INT7.1
void INTERRUPT DINTCH1_ISR(void)     // DMA-Channel 1
{

  for(;;);
}

// INT7.2
void INTERRUPT DINTCH2_ISR(void)     // DMA-Channel 2
{

  for(;;);
}

// INT7.3
void INTERRUPT DINTCH3_ISR(void)     // DMA-Channel 3
{

  for(;;);
}

// INT7.4
void INTERRUPT DINTCH4_ISR(void)     // DMA-Channel 4
{

  for(;;);
}


void INTERRUPT DINTCH5_ISR(void)     // DMA-Channel 5
{

  for(;;);
}

void INTERRUPT DINTCH6_ISR(void)     // DMA-Channel 6
{

  for(;;);
}

// -----------------------------------------------------------
// PIE Group 8 - MUXed into CPU INT8
// -----------------------------------------------------------

// INT8.1
void INTERRUPT I2CINT1A_ISR(void)     // I2C-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}


// INT8.3 - Reserved
// INT8.4 - Reserved
// INT8.5 - Reserved
// INT8.6 - Reserved
// INT8.7 - Reserved
// INT8.8 - Reserved

// -----------------------------------------------------------
// PIE Group 9 - MUXed into CPU INT9
// -----------------------------------------------------------

// INT9.1
void INTERRUPT SCITXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);

}
void INTERRUPT SCIRXINTA_ISR(void)     // SCI-A
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);

}


// INT9.3
void INTERRUPT LINA_INT0_ISR(void)     // LIN-A
{

  for(;;);

}
void INTERRUPT LINA_INT1_ISR(void)     // LIN-A
{

  for(;;);

}


// INT9.5
void INTERRUPT ECANA_INT0_ISR(void)     // I2C-A
{

  for(;;);
}
void INTERRUPT ECANA_INT1_ISR(void)     // I2C-A
{

  for(;;);
}


// INT9.7 - Reserved
// INT9.8 - Reserved

// -----------------------------------------------------------
// PIE Group 10 - MUXed into CPU INT10
// -----------------------------------------------------------

// INT10.3
void INTERRUPT ADCINT3_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}

// INT10.4
void INTERRUPT ADCINT4_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}

// INT10.5
void INTERRUPT ADCINT5_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}

// INT10.6
void INTERRUPT ADCINT6_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}

// INT10.7
void INTERRUPT ADCINT7_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}

// INT10.8
void INTERRUPT ADCINT8_ISR(void)    // ADC
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP10;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);
}


// -----------------------------------------------------------
// PIE Group 11 - MUXed into CPU INT11
// -----------------------------------------------------------

// -----------------------------------------------------------
// PIE Group 12 - MUXed into CPU INT12
// -----------------------------------------------------------

// INT12.1
void INTERRUPT XINT3_ISR(void)  // External interrupt 3
{
  // Insert ISR Code here

  // To receive more interrupts from this PIE group, acknowledge this interrupt
  // PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;

  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code
//  asm ("      ESTOP0");
  for(;;);

}
// INT12.2 - Reserved
// INT12.3 - Reserved
// INT12.4 - Reserved
// INT12.5 - Reserved
// INT12.6 - Reserved
// INT12.7 - Reserved
// INT12.8 - Reserved

//---------------------------------------------------------------------------
// Catch All Default ISRs:
//

void INTERRUPT EMPTY_ISR(void)  // Empty ISR - only does a return.
{

}

void INTERRUPT PIE_RESERVED(void)  // Reserved space.  For test.
{
//  asm ("      ESTOP0");
  for(;;);
}

void INTERRUPT rsvd_ISR(void)      // For test
{
//  asm ("      ESTOP0");
  for(;;);
}


void INTERRUPT TINT1_ISR(void)      // For test
{
//  asm ("      ESTOP0");
  for(;;);
}


void INTERRUPT TINT2_ISR(void)      // For test
{
//  asm ("      ESTOP0");
  for(;;);
}




//===========================================================================
// End of file.
//===========================================================================

