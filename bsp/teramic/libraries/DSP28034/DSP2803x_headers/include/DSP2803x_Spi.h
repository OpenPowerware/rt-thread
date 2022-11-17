
#ifndef DSP2803x_SPI_H
#define DSP2803x_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------------------------------------------------------
// SPI Individual Register Bit Definitions:
//
// SPI FIFO Transmit register bit    definitions:
struct  SPIFFTX_BITS {       // bit    description
   Uint32 TXFFIL:5;          // 4:0    Interrupt level
   Uint32 TXFFIENA:1;        // 5      Interrupt enable
   Uint32 TXFFINTCLR:1;      // 6      Clear INT flag ///??  0
   Uint32 TXFFINT:1;         // 7      INT flag
   Uint32 TXFFST:5;          // 12:8   FIFO status
   Uint32 TXFIFO:1;          // 13     FIFO reset
   Uint32 SPIFFENA:1;        // 14     Enhancement enable
   Uint32 SPIRST:1;          // 15     Reset SPI
   Uint32 rsvd:16;
};

union SPIFFTX_REG {
   Uint32               all;
   struct SPIFFTX_BITS  bit;
};

//--------------------------------------------
// SPI FIFO recieve register bit definitions:
//
//
struct  SPIFFRX_BITS {       // bits   description
   Uint32 RXFFIL:5;          // 4:0    Interrupt level
   Uint32 RXFFIENA:1;        // 5      Interrupt enable
   Uint32 RXFFINTCLR:1;      // 6      Clear INT flag
   Uint32 RXFFINT:1;         // 7      INT flag
   Uint32 RXFFST:5;          // 12:8   FIFO status
   Uint32 RXFIFORESET:1;     // 13     FIFO reset
   Uint32 RXFFOVFCLR:1;      // 14     Clear overflow
   Uint32 RXFFOVF:1;         // 15     FIFO overflow

   Uint32 rsvd:16;
};

union SPIFFRX_REG {
   Uint32               all;
   struct SPIFFRX_BITS  bit;
};

//--------------------------------------------
// SPI FIFO control register bit definitions:
//
//
struct  SPIFFCT_BITS {       // bits   description
   Uint32 TXDLY:8;           // 7:0    FIFO transmit delay
   Uint32 rsvd:8;            // 15:8   reserved
   Uint32 rsvd2:16;
};

union SPIFFCT_REG {
   Uint32               all;
   struct SPIFFCT_BITS  bit;
};

//---------------------------------------------
// SPI configuration register bit definitions:
//
//
struct  SPICCR_BITS {        // bits   description
   Uint32 SPICHAR:4;         // 3:0    Character length control
   Uint32 SPILBK:1;          // 4      Loop-back enable/disable
   Uint32 rsvd1:1;           // 5      reserved
   Uint32 CLKPOLARITY:1;     // 6      Clock polarity
   Uint32 SPISWRESET:1;      // 7      SPI SW Reset
   Uint32 rsvd2:8;           // 15:8   reserved
   Uint32 rsvd3:16;
};

union SPICCR_REG {
   Uint32              all;
   struct SPICCR_BITS  bit;
};

//-------------------------------------------------
// SPI operation control register bit definitions:
//
//
struct  SPICTL_BITS {        // bits   description
   Uint32 SPIINTENA:1;       // 0      Interrupt enable
   Uint32 TALK:1;            // 1      Master/Slave transmit enable
   Uint32 MASTER_SLAVE:1;    // 2      Network control mode
   Uint32 CLK_PHASE:1;       // 3      Clock phase select
   Uint32 OVERRUNINTENA:1;   // 4      Overrun interrupt enable
   Uint32 DMA_ENABLE:1;		//5
   Uint32 rsvd:10;           // 15:6   reserved
   Uint32 rsvd2:16;
};

union SPICTL_REG {
   Uint32              all;
   struct SPICTL_BITS  bit;
};

//--------------------------------------
// SPI status register bit definitions:
//
//
struct  SPISTS_BITS {        // bits   description
   Uint32 rsvd1:5;           // 4:0    reserved
   Uint32 BUFFULL_FLAG:1;    // 5      SPI transmit buffer full flag
   Uint32 INT_FLAG:1;        // 6      SPI interrupt flag
   Uint32 OVERRUN_FLAG:1;    // 7      SPI reciever overrun flag
   Uint32 rsvd2:8;           // 15:8   reserved
   Uint32 rsvd3:16;
};

union SPISTS_REG {
   Uint32              all;
   struct SPISTS_BITS  bit;
};

//------------------------------------------------
// SPI priority control register bit definitions:
//
//
struct  SPIPRI_BITS {        // bits   description
   Uint32 TRIWIRE:1;         // 0      3-wire mode select bit
   Uint32 STEINV:1;          // 1      SPISTE inversion bit
   Uint32 rsvd1:2;           // 3:2    reserved
   Uint32 FREE:1;            // 4      Free emulation mode control
   Uint32 SOFT:1;            // 5      Soft emulation mode control
   Uint32 rsvd2:1;          // 6      rsvd
   Uint32 rsvd3:9;           // 15:7   reserved

   Uint32 rsvd4:16;
};

union SPIPRI_REG {
   Uint32              all;
   struct SPIPRI_BITS  bit;
};

//---------------------------------------------------------------------------
// SPI Register File:
//
struct  SPI_REGS {
   union SPICCR_REG     SPICCR;      // Configuration register   00
   union SPICTL_REG     SPICTL;      // Operation control register  04
   union SPISTS_REG     SPISTS;      // Status register   08
   Uint32               SPIBRR;      // Baud Rate  0C
   Uint32               SPIRXEMU;    // Emulation buffer   10
   Uint32               SPIRXBUF;    // Serial input buffer  14
   Uint32               SPITXBUF;    // Serial output buffer  18
   Uint32               SPIDAT;      // Serial data   1C
   union SPIFFTX_REG    SPIFFTX;     // FIFO transmit register  20
   union SPIFFRX_REG    SPIFFRX;     // FIFO recieve register   24
   union SPIFFCT_REG    SPIFFCT;     // FIFO control register   28
   union SPIPRI_REG     SPIPRI;      // FIFO Priority control   2C
};

//---------------------------------------------------------------------------
// SPI External References & Function Declarations:
//P_de
//#define	 SpiaRegs (*P_SpiaRegs)
extern volatile struct SPI_REGS *const P_SpiaRegs;
extern volatile struct SPI_REGS SpiaRegs;

extern volatile struct SPI_REGS *const P_SpibRegs;
extern volatile struct SPI_REGS SpibRegs;


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  

//===========================================================================
// End of file.
//===========================================================================

