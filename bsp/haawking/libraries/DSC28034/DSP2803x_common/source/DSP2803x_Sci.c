//###########################################################################
//
// FILE:	DSP2803x_Sci.c
//
// TITLE:	DSP2803x SCI Initialization & Support Functions.
//
//###########################################################################
// $HAAWKING Release: DSP2803x Support Library V1.1.0 $
// $Release Date: 2022-07-09 04:53:39 $
// $Copyright:
// Copyright (C) 2019-2022 Beijing Haawking Technology Co.,Ltd - http://www.haawking.com/
//###########################################################################

#include "DSP2803x_Device.h"     //  Headerfile Include File
#include "DSP2803x_Examples.h"   //  Examples Include File

//---------------------------------------------------------------------------
// InitSci:
//---------------------------------------------------------------------------
// This function initializes the SCI(s) to a known state.
//
//void InitSci(void)
//{
	// Initialize SCI-A:

	//tbd...

//}

//---------------------------------------------------------------------------
// Example: InitSciGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SCI pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// Only one GPIO pin should be enabled for SCITXDA/B operation.
// Only one GPIO pin shoudl be enabled for SCIRXDA/B operation.
// Comment out other unwanted lines.

void InitSciGpio()
{
   InitSciaGpio();
}

void InitSciaGpio()
{
   EALLOW;

/* Enable internal pull-up for the selected pins */
/* Disable internal pull-up for the selected output pins
   to reduce power consumption. */
// Pull-ups can be enabled or disabled disabled by the user.

	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up for GPIO28 (SCIRXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;    // Enable pull-up for GPIO19 (SCIRXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;     // Enable pull-up for GPIO7  (SCIRXDA)

	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;	   // Disable pull-up for GPIO29 (SCITXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;	   // Disable pull-up for GPIO18 (SCITXDA)
//	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;	   // Disable pull-up for GPIO12 (SCITXDA)

/* Set qualification for selected pins to asynch only */
// Inputs are synchronized to SYSCLKOUT by default.
// This will select asynch (no qualification) for the selected pins.

	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDA)
//	GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 3;   // Asynch input GPIO7 (SCIRXDA)

/* Configure SCI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SCI functional pins.

	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 for SCIRXDA operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   // Configure GPIO19 for SCIRXDA operation
//	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;    // Configure GPIO7  for SCIRXDA operation

	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 for SCITXDA operation
//	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;   // Configure GPIO18 for SCITXDA operation
//	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 2;   // Configure GPIO12 for SCITXDA operation

    EDIS;
}

//===========================================================================
// End of file.
//===========================================================================
