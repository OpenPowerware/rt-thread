#ifndef DSP2803x_COMP_H
#define DSP2803x_COMP_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DSC28034

//===========================================================================
//  Comparator Register Bit Definitions
//

struct COMPCTL_BITS {            // bit       description
    Uint32   COMPDACEN:1;        // 0         Comparator/DAC  Enable
    Uint32   COMPSOURCE:1;       // 1         Source select for comparator inverting input
    Uint32   CMPINV:1;           // 2         Invert select for Comparator
    Uint32   QUALSEL:5;          // 7:03      Qualification Period for synchronized output of the comparator
    Uint32   SYNCSEL:1;          // 8         Synchronization select for output of the comparator
    Uint32   rsvd1:7;            // 15:09     reserved
    Uint32   rsvd2:16;
};

union COMPCTL_REG  {
    Uint32                         all;
    struct COMPCTL_BITS            bit;
};

struct COMPSTS_BITS {            // bit       description
    Uint32   COMPSTS:1;          // 0         Logical latched value of the comparator
    Uint32   rsvd1:15;           // 15:01     reserved
    Uint32   rsvd2:16;
};

union COMPSTS_REG  {
    Uint32                         all;
    struct COMPSTS_BITS            bit;
};

struct DACCTL_BITS
{
    Uint32 DACSOURCE:1;     //  0     0:DACVAL,1:RAMP
    Uint32 RAMPSOURCE:2;   //2:1
    Uint32 rsvd1:11;              //13:3	
    Uint32  FREE_SOFT:2;     //15:14
    Uint32  rsvd2:16;            // 31:16
};

union  DACCTL_REG
{
     Uint32 all;
     struct DACCTL_BITS bit;
};


struct DACVAL_BITS {             // bit       description
    Uint32   DACVAL:10;          // 9:00      DAC Value bit
    Uint32   rsvd1:6;            // 15:10     reserved
    Uint32   rsvd2:16;
};

union DACVAL_REG  {
    Uint32                         all;
    struct DACVAL_BITS             bit;
};


struct  RAMPMAXREF_ACTIVE_BITS
{
     Uint32 RAMPMAXREF_ACTIVE:16;// R
     Uint32 rsvd:16;
};


union  RAMPMAXREF_ACTIVE_REG 
{
     Uint32 all;
     struct  RAMPMAXREF_ACTIVE_BITS bit;
};


struct  RAMPMAXREF_SHDW_BITS
{
     Uint32 RAMPMAXREF_SHDW:16; // R
     Uint32 rsvd:16;
};

union  RAMPMAXREF_SHDW_REG
{
     Uint32 all;
     struct  RAMPMAXREF_SHDW_BITS bit;
};


struct RAMPDECVAL_ACTIVE_BITS
{
      Uint32 RAMPDECVAL_ACTIVE:16;	
      Uint32 rsvd:16;
};


union  RAMPDECVAL_ACTIVE_REG
{
     Uint32 all;
     struct RAMPDECVAL_ACTIVE_BITS bit;
};


struct RAMPDECVAL_SHDW_BITS
{
    Uint32 RAMPDECVAL_SHDW:16;	
    Uint32 rsvd:16;
};


union  RAMPDECVAL_SHDW_REG	
{
    Uint32 all;
    struct RAMPDECVAL_SHDW_BITS bit;
};

struct RAMPSTS_BITS
{
     Uint32 RAMPSTS:16;	// R
     Uint32 rsvd:16;
};


union  RAMPSTS_REG
{
     Uint32 all;
     struct RAMPSTS_BITS bit;
};


struct DACEX_BITS
{

    Uint32 DACCLKDIV:4;  //3:0 
    Uint32 rsvd1:11;      // 14:4
    Uint32 CMPRDY:1;  // 15  R  
    Uint32 rsvd2:16;
};

union  DACEX_REG
{
    Uint32 all;
    struct DACEX_BITS bit;
};


//===========================================================================
//  Comparator Register Definitions
//

struct COMP_REGS {
    union  COMPCTL_REG         COMPCTL;		//00
    union  COMPSTS_REG         COMPSTS;		//04
    union  DACCTL_REG		   DACCTL;		//08
    union  DACVAL_REG          DACVAL;		//0C
    union  RAMPMAXREF_ACTIVE_REG   RAMPMAXREF_ACTIVE;//10
    union  RAMPMAXREF_SHDW_REG	   RAMPMAXREF_SHDW;//14
    union  RAMPDECVAL_ACTIVE_REG   RAMPDECVAL_ACTIVE;//18
    union  RAMPDECVAL_SHDW_REG	   RAMPDECVAL_SHDW;//1C
    union  RAMPSTS_REG			   RAMPSTS;//20
    union  DACEX_REG	  		   DACEX;//24
};

//===========================================================================
//  Comparator External References and Function Declarations
//


extern volatile struct COMP_REGS  *const P_Comp1Regs;
extern volatile struct COMP_REGS  *const P_Comp2Regs;

extern volatile struct COMP_REGS  Comp1Regs;
extern volatile struct COMP_REGS  Comp2Regs;	

#endif

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif 
//===========================================================================
// End of file
//===========================================================================
