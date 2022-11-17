
#ifndef DSP2803x_DEVICE_H
#define DSP2803x_DEVICE_H



#define	_CPU_M0_BASE_ADDR	((uint32_t)0x00000000)
#define	_CPU_M1_BASE_ADDR	((uint32_t)0x00000800)
#define	_ADC_BASE_ADDR	    ((uint32_t)0x00001400) 
#define	_TIMER_PIE_BASE_ADDR	((uint32_t)0x00001800)
#define	_DMA_BASE_ADDR		((uint32_t)0x00001C00)
#define	_DEBUG_BASE_ADDR	((uint32_t)0x00002000)
#define	_APB_0_BASE_ADDR		((uint32_t)0x00009000)
#define	_APB_1_BASE_ADDR		((uint32_t)0x0000E000)
#define	_L0_BASE_ADDR		((uint32_t)0x00010000)
#define	_L1_BASE_ADDR		((uint32_t)0x00014000)
#define	_OTP_BASE_ADDR		((uint32_t)0x007AF000)

#define	_BOOT_ROM_BASE_ADDR	((uint32_t)0x007FC000)
#define	_CMP_BASE_ADDR		((uint32_t)0x00C000)

#define	_L0_DUAL_MAPPED_BASE_ADDR	((uint32_t)0x00010000)

/***********************************************************************
APB-0  PERIPHERALS
*************************************************************************/
#define _CAN_BASE_ADDR 		((uint32_t) 0x9000)
#define	_COMPARATOR_BASE_ADDR	((uint32_t)0x0000C000)
#define _LIN_BASE_ADDR		((uint32_t) 0xC400)

#define _HRCAP_BASE_ADDR	((uint32_t)0x0000CC00)
#define	_ECAP_BASE_ADDR		((uint32_t)0x0000D000)
#define	_EQEP_BASE_ADDR		((uint32_t)0x0000D400)
#define	_GPIO_BASE_ADDR		((uint32_t)0x0000D800)
#define	_SYS_CTRL_BASE_ADDR	((uint32_t) 0xDC00)



/***********************************************************************
APB-1  PERIPHERALS
*************************************************************************/
#define	_SCI_BASE_ADDR		((uint32_t)0x0000E000)
#define	_I2C_BASE_ADDR		((uint32_t)0x0000E400)
#define	_SPI_BASE_ADDR		((uint32_t)0x0000E800)
#define _HRCAP_BASE_ADDR    ((uint32_t)0x0000CC00)


/**************************************************************************
TIMER AND PIE PERIPHERALS
**************************************************************************/

#define	_TIMER_0_BASE_ADDR	(_TIMER_PIE_BASE_ADDR + 0)  
#define	_TIMER_1_BASE_ADDR	(_TIMER_PIE_BASE_ADDR + 0x10)
#define	_TIMER_2_BASE_ADDR	(_TIMER_PIE_BASE_ADDR + 0x20)


/*************************************************************************
	PIE PERIPHERALS
*************************************************************************/
#define	_PIE_BASE_ADDR	(_TIMER_PIE_BASE_ADDR + 0x100)
#define _VECTOR_PIE_BASE_ADDR  (_TIMER_PIE_BASE_ADDR + 0x200)
#define _PIE_EMU_REG_ADDR  ((uint32_t)0x1BC0)



#define _EPWM_BASE_ADDR	  ((uint32_t)0xB000)

/*************************************************************************
	EPWM PERIPHERALS
*************************************************************************/

#define	_EPWM_PER1_BASE_ADDR  _EPWM_BASE_ADDR
#define	_EPWM_PER2_BASE_ADDR  (_EPWM_PER1_BASE_ADDR + 0x100)
#define	_EPWM_PER3_BASE_ADDR  (_EPWM_PER2_BASE_ADDR + 0x100)
#define	_EPWM_PER4_BASE_ADDR  (_EPWM_PER3_BASE_ADDR + 0x100)

#define	_EPWM_PER5_BASE_ADDR  (_EPWM_PER3_BASE_ADDR + 0x100)
#define	_EPWM_PER6_BASE_ADDR  (_EPWM_PER3_BASE_ADDR + 0x100)
#define	_EPWM_PER7_BASE_ADDR  (_EPWM_PER3_BASE_ADDR + 0x100)




/*************************************************************************
	CMP PERIPHERALS
*************************************************************************/

#define	_CMP_PER1_ADDR	_CMP_BASE_ADDR
#define	_CMP_PER2_ADDR	(_CMP_PER1_ADDR + 0x80)
#define	_CMP_PER3_ADDR	(_CMP_PER2_ADDR + 0x100)

/******************************************************************
 HRCAP  PERIPHERALS
*****************************************************/
#define _HRCAP_PER1_ADDR	_HRCAP_BASE_ADDR
#define _HRCAP_PER2_ADDR	(_HRCAP_BASE_ADDR + 0x80)


#define _DMA_PER1_ADDR (_DMA_BASE_ADDR)
#define _DMA_PER2_ADDR (_DMA_BASE_ADDR + 0x50)
#define _DMA_PER3_ADDR (_DMA_BASE_ADDR + 0x50)
#define _DMA_PER4_ADDR (_DMA_BASE_ADDR + 0x50)

//0x7DFFF0 -0x7DFFFF
#define _CSM_PASSWORD_BASE_ADDR	((uint32_t)0x73FFF0)

#define _DEV_EMU_BASE_ADDR	(_SYS_CTRL_BASE_ADDR  +  0x80)

#define _FLASH_PER_REG_ADDR		((uint32_t)0x7AF800)



/*************************************************************************
	GPIO PERIPHERALS
*************************************************************************/

#define _GPIO_DATA_ADDR		(_GPIO_BASE_ADDR + 0x38)

#define _GPIO_INT_ADDR		(_GPIO_BASE_ADDR + 0x68)



/***************************************************************
	SPI PERIHERALS
*****************************************************************/
#define _SPIA_BASE_ADDR		_SPI_BASE_ADDR
#define _SPIB_BASE_ADDR		(_SPI_BASE_ADDR + 0x80)


#include <stdio.h>

#include "hx_rv32_dev.h"

#include "hx_rv32_type.h"


#define CODE_SECTION(v) __attribute__((section(v)))



#define   TARGET   1
//---------------------------------------------------------------------------
// User To Select Target Device:


#define   DSP28_28030PAG   0
#define   DSP28_28030PN    0

#define   DSP28_28031PAG   0
#define   DSP28_28031PN    0

#define   DSP28_28032PAG   0
#define   DSP28_28032PN    0

#define   DSP28_28033PAG   0
#define   DSP28_28033PN    0

#define   DSP28_28034PAG   0
#define   DSP28_28034PN    0

#define   DSP28_28035PAG   0
#define   DSP28_28035PN    TARGET



//---------------------------------------------------------------------------
// Common CPU Definitions:
//

extern  volatile unsigned int IFR;
extern  volatile unsigned int IER;



extern Uint32 ier_set(Uint32 group);
extern Uint32 ier_unset(Uint32 group);

extern Uint32 ifr_set(Uint32 group);
extern Uint32 ifr_unset(Uint32 group);


/*************************************************************
you can use it  like this

IER_ENABLE(M_INT1|M_INT3);
IER_DISABLE(M_INT1|M_INT3);

***********************************************************************/

#define IER_ENABLE(v)   ier_set(v)
#define IER_DISABLE(v)  ier_unset(v)

#define IFR_ENABLE(v)   ifr_set(v)
#define IFR_DISABLE(v)  ifr_unset(v)




#define EALLOW  asm("csrsi 0x7C1, 0x01")  // eallow  register id is  0x7C1 ,enable write spieacl register
#define	EDIS  	asm("csrci 0x7C1, 0x01")  // disable

#define  ESTOP0 asm(" ebreak"); //send debug call


#define M_INT1  0x0001
#define M_INT2  0x0002
#define M_INT3  0x0004
#define M_INT4  0x0008
#define M_INT5  0x0010
#define M_INT6  0x0020
#define M_INT7  0x0040
#define M_INT8  0x0080
#define M_INT9  0x0100
#define M_INT10 0x0200
#define M_INT11 0x0400
#define M_INT12 0x0800
#define M_INT13 0x1000
#define M_INT14 0x2000
#define M_DLOG  0x4000
#define M_RTOS  0x8000

#define BIT0    0x0001
#define BIT1    0x0002
#define BIT2    0x0004
#define BIT3    0x0008
#define BIT4    0x0010
#define BIT5    0x0020
#define BIT6    0x0040
#define BIT7    0x0080
#define BIT8    0x0100
#define BIT9    0x0200
#define BIT10   0x0400
#define BIT11   0x0800
#define BIT12   0x1000
#define BIT13   0x2000
#define BIT14   0x4000
#define BIT15   0x8000


//---------------------------------------------------------------------------
// Include All Peripheral Header Files:
//

#include "DSP2803x_Adc.h"                // ADC Registers
#include "DSP2803x_BootVars.h"           // Boot ROM Variables
#include "DSP2803x_Comp.h"               // Comparator Registers
#include "DSP2803x_CpuTimers.h"          // 32-bit CPU Timers
#include "DSP2803x_ECap.h"               // Enhanced Capture
#include "DSP2803x_EPwm.h"               // Enhanced PWM
#include "DSP2803x_Gpio.h"               // General Purpose I/O Registers
#include "DSP2803x_I2c.h"                // I2C Registers
#include "DSP2803x_PieCtrl.h"            // PIE Control Registers
#include "DSP2803x_PieVect.h"            // PIE Vector Table
#include "DSP2803x_Spi.h"                // SPI Registers
#include "DSP2803x_Sci.h"                // SCI Registers
#include "DSP2803x_Eqep.h"                // EQEP Registers
#include "DSP2803x_SysCtrl.h"            // System Control/Power Modes
#include "DSP2803x_Dma.h"           //  DMA Registers
#include "DSP2803x_Lin.h"           //  LIN Registers
#include "DSP2803x_ECan.h"           //  CAN Registers
#include "DSP2803x_HRCap.h"           //  HRCAP Registers


#if (DSP28_28035PN||DSP28_28034PN||DSP28_28033PN||DSP28_28032PN||DSP28_28031PN||DSP28_28030PN)
#define DSP28_COMP1 1
#define DSP28_COMP2 1
#define DSP28_COMP3 1
#define DSP28_EPWM1 1
#define DSP28_EPWM2 1
#define DSP28_EPWM3 1
#define DSP28_EPWM4 1
#define DSP28_EPWM5 1
#define DSP28_EPWM6 1
#define DSP28_EPWM7 1
#define DSP28_ECAP1 1
#define DSP28_EQEP1 1
#define DSP28_ECANA 1
#define DSP28_HRCAP1 1
#define DSP28_HRCAP2 1
#define DSP28_SPIA  1
#define DSP28_SPIB  1
#define DSP28_SCIA  1
#define DSP28_I2CA  1
#define DSP28_LINA  1
#endif

#if (DSP28_28035PAG||DSP28_28034PAG||DSP28_28033PAG||DSP28_28032PAG||DSP28_28031PAG||DSP28_28030PAG)
#define DSP28_COMP1 1
#define DSP28_COMP2 1
#define DSP28_COMP3 1
#define DSP28_EPWM1 1
#define DSP28_EPWM2 1
#define DSP28_EPWM3 1
#define DSP28_EPWM4 1
#define DSP28_EPWM5 1
#define DSP28_EPWM6 1
#define DSP28_EPWM7 0
#define DSP28_ECAP1 1
#define DSP28_EQEP1 1
#define DSP28_ECANA 1
#define DSP28_HRCAP1 1
#define DSP28_HRCAP2 1
#define DSP28_SPIA  1
#define DSP28_SPIB  0
#define DSP28_SCIA  1
#define DSP28_I2CA  1
#define DSP28_LINA  1
#endif

// Timer definitions based on System Clock
// 60 MHz devices 
	#define      mSec0_5          30000           // 0.5 mS
	#define      mSec0_75         45000           // 0.75 mS
	#define      mSec1            60000           // 1.0 mS
	#define      mSec2            120000          // 2.0 mS
	#define      mSec5            300000          // 5.0 mS
	#define      mSec7_5          450000          // 7.5 mS
	#define      mSec10           600000          // 10 mS
	#define      mSec20           1200000         // 20 mS
	#define      mSec50           3000000         // 50 mS
	#define      mSec75           4500000         // 75 mS
	#define      mSec100          6000000         // 100 mS
	#define      mSec200          12000000        // 200 mS
	#define      mSec500          30000000        // 500 mS
	#define      mSec750          45000000        // 750 mS
	#define      mSec1000         60000000        // 1000 mS
	#define      mSec2000         120000000       // 2000 mS
	#define      mSec5000         300000000       // 5000 mS



extern volatile Uint32   IER;
extern volatile Uint32   IFR;


#endif

