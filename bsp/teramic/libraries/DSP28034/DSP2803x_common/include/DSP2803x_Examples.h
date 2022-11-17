
#ifndef DSP2803x_EXAMPLES_H
#define DSP2803x_EXAMPLES_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------------------------
      Specify the PLL control register (PLLCR) and divide select (DIVSEL) value.
-----------------------------------------------------------------------------*/

//#define DSP28_DIVSEL   0 // Enable /4 for SYSCLKOUT
//#define DSP28_DIVSEL   1 // Disable /4 for SYSCKOUT
//#define DSP28_DIVSEL   2 // Enable /2 for SYSCLKOUT
#define DSP28_DIVSEL   3 // Enable /1 for SYSCLKOUT


//#define DSP28_PLLCR   21    
//#define DSP28_PLLCR   20   
//#define DSP28_PLLCR   19    
//#define DSP28_PLLCR   18    
//#define DSP28_PLLCR   17    
//#define DSP28_PLLCR   16    
//#define DSP28_PLLCR   15    
//#define DSP28_PLLCR   14    
//#define DSP28_PLLCR   13   
//#define DSP28_PLLCR   12    
//#define DSP28_PLLCR   11
#define DSP28_PLLCR   10       // Uncomment for 120 MHz devices [120 MHz = (12MHz * 10)/1]
//#define DSP28_PLLCR    9
//#define DSP28_PLLCR    8     // Uncomment for 96 Mhz devices [96 Mhz = (12MHz * 8)/1]
//#define DSP28_PLLCR    7
//#define DSP28_PLLCR    0   //PLL is bypassed in this mode

//----------------------------------------------------------------------------

/*-----------------------------------------------------------------------------
      Specify the clock rate of the CPU (SYSCLKOUT) in nS.

      Take into account the input clock frequency and the PLL multiplier
      selected in step 1.

      Use one of the values provided, or define your own.
      The trailing L is required tells the compiler to treat
      the number as a 64-bit value.

      Only one statement should be uncommented.
      Example 1: 120 MHz devices:
                 CLKIN is a 12 MHz crystal or internal 12 MHz oscillator

                 In step 1 the user specified PLLCR = 10 DIVSEL=3 for a
                 120 MHz CPU clock (SYSCLKOUT = 120 MHz).

                 In this case, the CPU_RATE will be 8.333L
                 Uncomment the line: #define CPU_RATE 8.333L

      Example 2: 96 MHz devices:
                 CLKIN is a 12 MHz crystal or internal 12 MHz oscillator

                 In step 1 the user specified PLLCR = 8 for a
                 96 MHz CPU clock (SYSCLKOUT = 96 MHz).

                 In this case, the CPU_RATE will be 10.417L
                 Uncomment the line: #define CPU_RATE 10.417L


      Example 3: 60 MHz devices:
                 CLKIN is a 12 MHz crystal or internal 12 MHz oscillator

                 In step 1 the user specified PLLCR = 10 DIVSEL=2 for a
                 60 MHz CPU clock (SYSCLKOUT = 60 MHz).

                 In this case, the CPU_RATE will be 16.667L
                 Uncomment the line: #define CPU_RATE 16.667L

-----------------------------------------------------------------------------*/
#define CPU_RATE   8.333L   // for a 120MHz CPU clock speed (SYSCLKOUT) 
//#define CPU_RATE   10.417L   // for a 96MHz CPU clock speed (SYSCLKOUT)   
//#define CPU_RATE   16.667L   // for a 60MHz CPU clock speed (SYSCLKOUT) 
//#define CPU_RATE   20.000L   // for a 50MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   25.000L   // for a 40MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   33.333L   // for a 30MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   41.667L   // for a 24MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   50.000L   // for a 20MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   66.667L   // for a 15MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   83.33L   // for a 12MHz CPU clock speed (SYSCLKOUT) 
//#define CPU_RATE  100.000L   // for a 10MHz CPU clock speed  (SYSCLKOUT)


void OSC_LDO_cal(void);

// The following pointer to a function call internal oscillators
#define Device_cal (void   (*)(void))OSC_LDO_cal

//---------------------------------------------------------------------------
// Include Example Header Files:
//

#include "DSP2803x_GlobalPrototypes.h"         // Prototypes for global functions within the
#include "DSP2803x_EPwm_defines.h"             // Macros used for PWM examples.
#include "DSP2803x_I2c_defines.h"                   // Macros used for I2C examples.


#ifndef DSP28_BIOS

#include "DSP2803x_DefaultISR.h"

#endif

extern void _DSP2803x_usDelay(uint32 data);

// DO NOT MODIFY THIS LINE.
#define DELAY_US(A)  _DSP2803x_usDelay((((long double)A * 1000.0L) / (long double)CPU_RATE) - 15.0L) // range A: 1 ~ 34000


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif 

//===========================================================================
// End of file.
//===========================================================================
