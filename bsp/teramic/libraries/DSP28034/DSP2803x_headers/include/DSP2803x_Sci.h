

#ifndef DSP2803x_SCI_H
#define DSP2803x_SCI_H

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------
// SCI Individual Register Bit Definitions

//----------------------------------------------------------
// SCICCR communication control register bit definitions:
//

struct  SCICCR_BITS {        // bit    description
   Uint32 SCICHAR:3;         // 2:0    Character length control
   Uint32 ADDRIDLE_MODE:1;   // 3      ADDR/IDLE Mode control
   Uint32 LOOPBKENA:1;       // 4      Loop Back enable
   Uint32 PARITYENA:1;       // 5      Parity enable
   Uint32 PARITY:1;          // 6      Even or Odd Parity
   Uint32 STOPBITS:1;        // 7      Number of Stop Bits
   Uint32 rsvd1:8;           // 15:8   reserved
   Uint32 rsvd2:16;			//rsvd  16-31
};

union SCICCR_REG {
   Uint32              all;
   struct SCICCR_BITS  bit;
};

//-------------------------------------------
// SCICTL1 control register 1 bit definitions:
//

struct  SCICTL1_BITS {       // bit    description
   Uint32 RXENA:1;           // 0      SCI receiver enable
   Uint32 TXENA:1;           // 1      SCI transmitter enable
   Uint32 SLEEP:1;           // 2      SCI sleep
   Uint32 TXWAKE:1;          // 3      Transmitter wakeup method
   Uint32 rsvd:1;            // 4      reserved
   Uint32 SWRESET:1;         // 5      Software reset
   Uint32 RXERRINTENA:1;     // 6      Recieve interrupt enable
   Uint32 rsvd1:9;           // 15:7   reserved
   Uint32 rsvd2:16;			// 16:31   16-31

};

union SCICTL1_REG {
   Uint32               all;
   struct SCICTL1_BITS  bit;
};

//---------------------------------------------
// SCICTL2 control register 2 bit definitions:
//

struct  SCICTL2_BITS {       // bit    description
   Uint32 TXINTENA:1;        // 0      Transmit interrupt enable
   Uint32 RXBKINTENA:1;      // 1      Receiver-buffer break enable
   Uint32 rsvd:4;            // 5:2    reserved
   Uint32 TXEMPTY:1;         // 6      Transmitter empty flag
   Uint32 TXRDY:1;           // 7      Transmitter ready flag
   Uint32 rsvd1:8;           // 15:8   reserved
   Uint32 rsvd2:16;			// 16 -31
};

union SCICTL2_REG {
   Uint32               all;
   struct SCICTL2_BITS  bit;
};

//---------------------------------------------------
// SCIRXST Receiver status register bit definitions:
//

struct  SCIRXST_BITS {       // bit    description
   Uint32 rsvd:1;            // 0      reserved
   Uint32 RXWAKE:1;          // 1      Receiver wakeup detect flag
   Uint32 PE:1;              // 2      Parity error flag
   Uint32 OE:1;              // 3      Overrun error flag
   Uint32 FE:1;              // 4      Framing error flag
   Uint32 BRKDT:1;           // 5      Break-detect flag
   Uint32 RXRDY:1;           // 6      Receiver ready flag
   Uint32 RXERROR:1;         // 7      Receiver error flag
   Uint32 rsvd1:24;			//rsvd	8-31
};

union SCIRXST_REG {
   Uint32               all;
   struct SCIRXST_BITS  bit;
};

//----------------------------------------------------
// SCIRXBUF Receiver Data Buffer with FIFO bit definitions:
//

struct  SCIRXBUF_BITS {      // bits   description
   Uint32 RXDT:8;            // 7:0    Receive word
   Uint32 rsvd:6;            // 13:8   reserved
   Uint32 SCIFFPE:1;         // 14     SCI PE error in FIFO mode
   Uint32 SCIFFFE:1;         // 15     SCI FE error in FIFO mode
   Uint32 rsvd1:16;			// 16 -31
};

union SCIRXBUF_REG {
   Uint32                all;
   struct SCIRXBUF_BITS  bit;
};

//--------------------------------------------------
// SCIPRI Priority control register bit definitions:
//
//

struct  SCIPRI_BITS {        // bit    description
   Uint32 rsvd:3;            // 2:0    reserved
   Uint32 FREE:1;            // 3      Free emulation suspend mode
   Uint32 SOFT:1;            // 4      Soft emulation suspend mode
   Uint32 rsvd1:3;           // 7:5    reserved
   Uint32 rsvd2:24;			//rsvd  8-31
};

union SCIPRI_REG {
   Uint32              all;
   struct SCIPRI_BITS  bit;
};

//-------------------------------------------------
// SCI FIFO Transmit register bit definitions:
//
//

struct  SCIFFTX_BITS {       // bit    description
   Uint32 TXFFIL:5;          // 4:0    Interrupt level
   Uint32 TXFFIENA:1;        // 5      Interrupt enable
   Uint32 TXFFINTCLR:1;      // 6      Clear INT flag
   Uint32 TXFFINT:1;         // 7      INT flag
   Uint32 TXFFST:5;          // 12:8   FIFO status
   Uint32 TXFIFOXRESET:1;    // 13     FIFO reset
   Uint32 SCIFFENA:1;        // 14     Enhancement enable
   Uint32 SCIRST:1;          // 15     SCI reset rx/tx channels
   Uint32 rsvd:16;			//rsvd		16-31
};

union SCIFFTX_REG {
   Uint32               all;
   struct SCIFFTX_BITS  bit;
};

//------------------------------------------------
// SCI FIFO recieve register bit definitions:
//
//

struct  SCIFFRX_BITS {       // bits   description
   Uint32 RXFFIL:5;          // 4:0    Interrupt level
   Uint32 RXFFIENA:1;        // 5      Interrupt enable
   Uint32 RXFFINTCLR:1;      // 6      Clear INT flag
   Uint32 RXFFINT:1;         // 7      INT flag
   Uint32 RXFFST:5;          // 12:8   FIFO status
   Uint32 RXFIFORESET:1;     // 13     FIFO reset
   Uint32 RXFFOVRCLR:1;      // 14     Clear overflow
   Uint32 RXFFOVF:1;         // 15     FIFO overflow
   Uint32 rsvd:16;			// rsvd	16-31
};

union SCIFFRX_REG {
   Uint32               all;
   struct SCIFFRX_BITS  bit;
};

// SCI FIFO control register bit definitions:
struct  SCIFFCT_BITS {     // bits   description
   Uint32 FFTXDLY:8;         // 7:0    FIFO transmit delay
   Uint32 rsvd:5;            // 12:8   reserved
   Uint32 CDC:1;             // 13     Auto baud mode enable
   Uint32 ABDCLR:1;          // 14     Auto baud clear
   Uint32 ABD:1;             // 15     Auto baud detect
   Uint32 rsvd2:16;			//16	16-31
};

union SCIFFCT_REG {
   Uint32               all;
   struct SCIFFCT_BITS  bit;
};
   #ifdef    DSC28036
struct SCIRXTOCTL_BITS {
     Uint32        RXTOFLAG:1;  //0  Write 1 clear
     Uint32        rsvd1:14;        //1-14 rsvd
     Uint32        RXTOEN:1;  //15
     Uint32         rsvd2:16;    //16-31
};

union  SCIRXTOCTL_REG {
    Uint32             all;
    struct  SCIRXTOCTL_BITS bit;
};

struct  SCIRXTOCNT_BITS {
     Uint32        RXTOCNT:16;  //0-15
     Uint32        rsvd1:16;        //16-31
};

union  SCIRXTOCNT_REG {
    Uint32             all;
    struct  SCIRXTOCNT_BITS bit;
};

   #endif
//---------------------------------------------------------------------------
// SCI Register File:
//
struct  SCI_REGS {
   union SCICCR_REG     SCICCR;     // Communications control register  0
   union SCICTL1_REG    SCICTL1;    // Control register 1				4
   Uint32               SCIHBAUD;   // Baud rate (high) register		8
   Uint32               SCILBAUD;   // Baud rate (low) register			c
   union SCICTL2_REG    SCICTL2;    // Control register 2				10
   union SCIRXST_REG    SCIRXST;    // Recieve status register			14
   Uint32               SCIRXEMU;   // Recieve emulation buffer register		18
   union SCIRXBUF_REG   SCIRXBUF;   // Recieve data buffer				1c
   Uint32               SCITXBUF;   // Transmit data buffer				20
   union SCIFFTX_REG    SCIFFTX;    // FIFO transmit register			24
   union SCIFFRX_REG    SCIFFRX;    // FIFO recieve register			28
   union SCIFFCT_REG    SCIFFCT;    // FIFO control register			2c
   union SCIPRI_REG     SCIPRI;     // FIFO Priority control			30
   #ifdef    DSC28036
   union  SCIRXTOCTL_REG    SCIRXTOCTL;
   union  SCIRXTOCNT_REG    SCIRXTOCNT;
   #endif


};

//---------------------------------------------------------------------------
// SCI External References & Function Declarations:
//

extern volatile struct SCI_REGS *const P_SciaRegs;
extern volatile struct SCI_REGS  SciaRegs;


#ifdef __cplusplus
}
#endif /* extern "C" */


#endif  

//===========================================================================
// End of file.
//===========================================================================

