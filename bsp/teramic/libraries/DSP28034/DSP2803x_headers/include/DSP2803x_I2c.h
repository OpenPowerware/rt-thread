#ifndef DSP2803x_I2C_H
#define DSP2803x_I2C_H



#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------
// I2C interrupt vector register bit definitions
struct I2CISRC_BITS {         // bits   description
   Uint32 INTCODE:3;          // 2:0    Interrupt code
   Uint32 rsvd1:29;           // 31:3   reserved
};

union I2CISRC_REG {
   Uint32                 all;
   struct I2CISRC_BITS    bit;
};

//----------------------------------------------------
// I2C interrupt mask register bit definitions
struct I2CIER_BITS {          // bits   description
   Uint32 ARBL:1;               // 0      Arbitration lost interrupt
   Uint32 NACK:1;             // 1      No ack interrupt
   Uint32 ARDY:1;             // 2      Register access ready interrupt
   Uint32 RRDY:1;             // 3      Recieve data ready interrupt
   Uint32 XRDY:1;             // 4      Transmit data ready interrupt
   Uint32 SCD:1;              // 5      Stop condition detection
   Uint32 AAS:1;              // 6      Address as slave
   Uint32 rsvd:25;             // 31:7   reserved
};

union I2CIER_REG {
   Uint32                 all;
   struct I2CIER_BITS     bit;
};

//----------------------------------------------------
// I2C status register bit definitions
struct I2CSTR_BITS {          // bits   description
   Uint32 ARBL:1;               // 0      Arbitration lost interrupt
   Uint32 NACK:1;             // 1      No ack interrupt
   Uint32 ARDY:1;             // 2      Register access ready interrupt
   Uint32 RRDY:1;             // 3      Recieve data ready interrupt
   Uint32 XRDY:1;             // 4      Transmit data ready interrupt
   Uint32 SCD:1;              // 5      Stop condition detection
   Uint32 rsvd1:2;            // 7:6    reserved
   Uint32 AD0:1;              // 8      Address Zero
   Uint32 AAS:1;              // 9      Address as slave
   Uint32 XSMT:1;             // 10     XMIT shift empty
   Uint32 RSFULL:1;           // 11     Recieve shift full
   Uint32 BB:1;               // 12     Bus busy
   Uint32 NACKSNT:1;          // 13     A no ack sent
   Uint32 SDIR:1;             // 14     Slave direction
   Uint32 rsvd2:17;            // 15     reserved
};

union I2CSTR_REG {
   Uint32                 all;
   struct I2CSTR_BITS     bit;
};

//----------------------------------------------------
// I2C mode control register bit definitions
struct I2CMDR_BITS {          // bits   description
   Uint32 BC:3;               // 2:0    Bit count
   Uint32 FDF:1;              // 3      Free data format
   Uint32 STB:1;              // 4      Start byte
   Uint32 IRS:1;              // 5      I2C Reset not
   Uint32 DLB:1;              // 6      Digital loopback
   Uint32 RM:1;               // 7      Repeat mode
   Uint32 XA:1;               // 8      Expand address
   Uint32 TRX:1;              // 9      Transmitter/reciever
   Uint32 MST:1;              // 10     Master/slave
   Uint32 STP:1;              // 11     Stop condition
   Uint32 rsvd1:1;            // 12     reserved
   Uint32 STT:1;              // 13     Start condition
   Uint32 FREE:1;             // 14     Emulation mode
   Uint32 NACKMOD:1;          // 15     No Ack mode
   Uint32 rsvd2:16;			  // 31:16   reserved
};

union I2CMDR_REG {
   Uint32                 all;
   struct I2CMDR_BITS     bit;
};

//--------------------------------------------------
// I2C extended mode control register bit definitions
struct I2CEMDR_BITS       // bits   description
{
	uint32 BCM:1;        //0 Bit count    
	uint32 rsvd1:31;    //31:1  reserved
};

union I2CEMDR_REG
{
	uint32                 all;
	struct  I2CEMDR_BITS   bit;
};


//----------------------------------------------------
// I2C pre-scaler register bit definitions
struct I2CPSC_BITS {         // bits   description
   Uint32 IPSC:8;            // 7:0    pre-scaler
   Uint32 rsvd1:8;           // 15:8   reserved
   Uint32 rsvd2:16;			 // 31:16   reserved
};

union I2CPSC_REG {
   Uint32                 all;
   struct I2CPSC_BITS     bit;
};

//----------------------------------------------------
// TX FIFO control register bit definitions
struct I2CFFTX_BITS {         // bits   description
   Uint32 TXFFIL:5;           // 4:0    FIFO interrupt level
   Uint32 TXFFIENA:1;         // 5      FIFO interrupt enable/disable
   Uint32 TXFFINTCLR:1;       // 6      FIFO clear
   Uint32 TXFFINT:1;          // 7      FIFO interrupt flag
   Uint32 TXFFST:5;           // 12:8   FIFO level status
   Uint32 TXFFRST:1;          // 13     FIFO reset
   Uint32 I2CFFEN:1;          // 14     enable/disable TX & RX FIFOs
   Uint32 rsvd1:17;            // 31:15     reserved

};

union I2CFFTX_REG {
   Uint32                 all;
   struct I2CFFTX_BITS    bit;
};

//----------------------------------------------------
// RX FIFO control register bit definitions 
struct I2CFFRX_BITS {         // bits   description
   Uint32 RXFFIL:5;           // 4:0    FIFO interrupt level
   Uint32 RXFFIENA:1;         // 5      FIFO interrupt enable/disable
   Uint32 RXFFINTCLR:1;       // 6      FIFO clear
   Uint32 RXFFINT:1;          // 7      FIFO interrupt flag
   Uint32 RXFFST:5;           // 12:8   FIFO level
   Uint32 RXFFRST:1;          // 13     FIFO reset
   Uint32 rsvd1:18;            // 31:14  reserved
};

union I2CFFRX_REG {
   Uint32                 all;
   struct I2CFFRX_BITS    bit;
};

//----------------------------------------------------

struct I2C_REGS {
   Uint32              I2COAR;    // Own address register		0
   union  I2CIER_REG   I2CIER;    // Interrupt enable			4
   union  I2CSTR_REG   I2CSTR;    // Interrupt status			8
   Uint32              I2CCLKL;   // Clock divider low			c
   Uint32              I2CCLKH;   // Clock divider high			10
   Uint32              I2CCNT;    // Data count					14
   Uint32              I2CDRR;    // Data recieve				18
   Uint32              I2CSAR;    // Slave address				1c
   Uint32              I2CDXR;    // Data transmit				20
   union  I2CMDR_REG   I2CMDR;    // Mode						24
   union  I2CISRC_REG  I2CISRC;   // Interrupt source			28
   union  I2CEMDR_REG  I2CEMDR;	 //								2c
   union  I2CPSC_REG   I2CPSC;    // Pre-scaler					30
   union  I2CFFTX_REG  I2CFFTX;   // Transmit FIFO				34
   union  I2CFFRX_REG  I2CFFRX;   // Recieve FIFO				38
};



//---------------------------------------------------------------------------
// External References & Function Declarations:
//
extern volatile struct I2C_REGS *const P_I2caRegs ;
extern volatile struct I2C_REGS I2caRegs ;


#ifdef __cplusplus
}
#endif /* extern "C" */


#endif 

//===========================================================================
// End of file.
//===========================================================================
