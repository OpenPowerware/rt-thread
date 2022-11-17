

#ifndef DSP2803x_PIE_CTRL_H
#define DSP2803x_PIE_CTRL_H


#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------
// PIE Control Register Bit Definitions:
//
// PIECTRL: Register bit definitions:
struct PIECTRL_BITS {      // bits description
   Uint32  ENPIE:1;        // 0    Enable PIE block
   Uint32  PIEVECT:15;     // 15:1 Fetched vector address
   Uint32  rsvd1:16;         //31:16 reserved
};

union PIECTRL_REG {
   Uint32               all;
   struct PIECTRL_BITS  bit;
};  

// PIEIER: Register bit definitions:
struct PIEIER_BITS {       // bits description
   Uint32 INTx1:1;         // 0    INTx.1
   Uint32 INTx2:1;         // 1    INTx.2
   Uint32 INTx3:1;         // 2    INTx.3
   Uint32 INTx4:1;         // 3    INTx.4
   Uint32 INTx5:1;         // 4    INTx.5
   Uint32 INTx6:1;         // 5    INTx.6
   Uint32 INTx7:1;         // 6    INTx.7
   Uint32 INTx8:1;         // 7    INTx.8
   Uint32 rsvd:24;          // 31:8 reserved
};

union PIEIER_REG {
   Uint32              all;
   struct PIEIER_BITS  bit;
}; 

// PIEIFR: Register bit definitions:
struct PIEIFR_BITS {       // bits description
   Uint32 INTx1:1;         // 0    INTx.1
   Uint32 INTx2:1;         // 1    INTx.2
   Uint32 INTx3:1;         // 2    INTx.3
   Uint32 INTx4:1;         // 3    INTx.4
   Uint32 INTx5:1;         // 4    INTx.5
   Uint32 INTx6:1;         // 5    INTx.6
   Uint32 INTx7:1;         // 6    INTx.7
   Uint32 INTx8:1;         // 7    INTx.8
   Uint32 rsvd:24;          // 31:8 reserved
};

union PIEIFR_REG {
   Uint32              all;
   struct PIEIFR_BITS  bit;
};

// PIEACK: Register bit definitions:
struct PIEACK_BITS {       // bits description
   Uint32 ACK1:1;          // 0    Acknowledge PIE interrupt group 1
   Uint32 ACK2:1;          // 1    Acknowledge PIE interrupt group 2
   Uint32 ACK3:1;          // 2    Acknowledge PIE interrupt group 3
   Uint32 ACK4:1;          // 3    Acknowledge PIE interrupt group 4
   Uint32 ACK5:1;          // 4    Acknowledge PIE interrupt group 5
   Uint32 ACK6:1;          // 5    Acknowledge PIE interrupt group 6
   Uint32 ACK7:1;          // 6    Acknowledge PIE interrupt group 7
   Uint32 ACK8:1;          // 7    Acknowledge PIE interrupt group 8
   Uint32 ACK9:1;          // 8    Acknowledge PIE interrupt group 9
   Uint32 ACK10:1;         // 9    Acknowledge PIE interrupt group 10
   Uint32 ACK11:1;         // 10   Acknowledge PIE interrupt group 11
   Uint32 ACK12:1;         // 11   Acknowledge PIE interrupt group 12
   Uint32 rsvd:20;          // 31:12 reserved
};

union PIEACK_REG {
   Uint32              all;
   struct PIEACK_BITS  bit;
};

struct XINTCR_BITS {
   Uint32 ENABLE:1;   //Enable XINCTR
   Uint32 rsvd1:1;
   Uint32 POLARITY:2;   //Polarity of XINCTR
   Uint32 rsvd2:28;
};

union XINTCR_REG {
   Uint32              all;
   struct XINTCR_BITS  bit;
};


struct 	EMU_BITS
{
    Uint32 EmuKey:16;//0-15
    Uint32 EmuBMode:16;//16 - 31
};


union  EMU_REGS{
    Uint32              all;
    struct 	EMU_BITS  bit;
};


struct PieEmu_REGS
{
    union  EMU_REGS Emu;
};



//---------------------------------------------------------------------------
// PIE Control Register File:
//
struct PIE_CTRL_REGS {
   union PIECTRL_REG PIECTRL;       // PIE control register      00
   union PIEACK_REG  PIEACK;        // PIE acknowledge			 04
   union PIEIER_REG  PIEIER1;       // PIE INT1 IER register     08
   union PIEIFR_REG  PIEIFR1;       // PIE INT1 IFR register     0C
   union PIEIER_REG  PIEIER2;       // PIE INT2 IER register     10
   union PIEIFR_REG  PIEIFR2;       // PIE INT2 IFR register     14
   union PIEIER_REG  PIEIER3;       // PIE INT3 IER register     18
   union PIEIFR_REG  PIEIFR3;       // PIE INT3 IFR register	 1C
   union PIEIER_REG  PIEIER4;       // PIE INT4 IER register     20      
   union PIEIFR_REG  PIEIFR4;       // PIE INT4 IFR register     24
   union PIEIER_REG  PIEIER5;       // PIE INT5 IER register     28
   union PIEIFR_REG  PIEIFR5;       // PIE INT5 IFR register     2C
   union PIEIER_REG  PIEIER6;       // PIE INT6 IER register     30
   union PIEIFR_REG  PIEIFR6;       // PIE INT6 IFR register     34
   union PIEIER_REG  PIEIER7;       // PIE INT7 IER register     38
   union PIEIFR_REG  PIEIFR7;       // PIE INT7 IFR register     3C
   union PIEIER_REG  PIEIER8;       // PIE INT8 IER register     40
   union PIEIFR_REG  PIEIFR8;       // PIE INT8 IFR register     44
   union PIEIER_REG  PIEIER9;       // PIE INT9 IER register     48 
   union PIEIFR_REG  PIEIFR9;       // PIE INT9 IFR register     4C
   union PIEIER_REG  PIEIER10;      // PIE INT10 IER register    50
   union PIEIFR_REG  PIEIFR10;      // PIE INT10 IFR register    54
   union PIEIER_REG  PIEIER11;      // PIE INT11 IER register    58
   union PIEIFR_REG  PIEIFR11;      // PIE INT11 IFR register    5C
   union PIEIER_REG  PIEIER12;      // PIE INT12 IER register    60
   union PIEIFR_REG  PIEIFR12;      // PIE INT12 IFR register    64 

};     

struct 	XINTRUPT_REGS {

   union XINTCR_REG  XINT1CR;		// XINT1 configuration register   68
   union XINTCR_REG  XINT2CR;		// XINT2 configuration register   6C
   union XINTCR_REG  XINT3CR;		// XINT3 configuration register   70
   Uint32  XINT1CTR;				// XINT1 counter register         74
   Uint32  XINT2CTR;				// XINT2 counter register         78
   Uint32  XINT3CTR;				// XINT3 counter register         7C

};

#define PIEACK_GROUP1   0x0001
#define PIEACK_GROUP2   0x0002
#define PIEACK_GROUP3   0x0004
#define PIEACK_GROUP4   0x0008
#define PIEACK_GROUP5   0x0010
#define PIEACK_GROUP6   0x0020
#define PIEACK_GROUP7   0x0040
#define PIEACK_GROUP8   0x0080
#define PIEACK_GROUP9   0x0100
#define PIEACK_GROUP10  0x0200
#define PIEACK_GROUP11  0x0400
#define PIEACK_GROUP12  0x0800

//---------------------------------------------------------------------------
// PIE Control Registers External References & Function Declarations:
//

extern volatile struct PIE_CTRL_REGS *const P_PieCtrlRegs;
extern volatile struct PIE_CTRL_REGS PieCtrlRegs;


extern volatile struct XINTRUPT_REGS *const P_XIntruptRegs;
extern volatile struct XINTRUPT_REGS XIntruptRegs;

//#define XIntruptRegs  PieCtrlRegs


extern volatile struct PieEmu_REGS *const P_PieEmuRegs;
extern volatile struct PieEmu_REGS PieEmuRegs;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif

//===========================================================================
// End of file.
//===========================================================================
