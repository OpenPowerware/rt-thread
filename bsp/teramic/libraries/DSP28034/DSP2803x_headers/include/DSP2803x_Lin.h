
#ifndef DSP2803x_LIN_H
#define DSP2803x_LIN_H

#ifdef __cplusplus
extern "C" {
#endif

//
// LIN Registers
//

//
// Global Control Register 0 (SCIGCR0) bit definitions
//
struct SCIGCR0_BITS {      // bit    description
    Uint32  RESET:1;       // 0      LIN Module reset bit
    Uint32  rsvd1:15;      // 15:1   reserved
    Uint32  rsvd2:16;      // 31:16  reserved
};

//
// Allow access to the bit fields or entire register
//
union SCIGCR0_REG {
    Uint32               all;
    struct SCIGCR0_BITS  bit;
};

//
// Global Control Register 1 (SCIGCR1) bit definitions
//
struct SCIGCR1_BITS {      // bit    description
    Uint32  COMMMODE:1;    // 0 SCI/LIN communications mode bit
    
    //
    // 1 SCI timing mode bit. Should be set to 1 for SCI mode.
    //
    Uint32  TIMINGMODE:1;  
    
    Uint32  PARITYENA:1;   // 2 Parity enable
    Uint32  PARITY:1;      // 3 SCI parity odd/even selection
    Uint32  STOP:1;        // 4 SCI number of stop bits
    Uint32  CLK_MASTER:1;  // 5 LIN Master/Slave selection and SCI clock enable
    Uint32  LINMODE:1;     // 6 LIN Mode enable/disable
    Uint32  SWnRST:1;      // 7 Software reset
    Uint32  SLEEP:1;       // 8 SCI sleep (SCI compatibility mode)
    Uint32  ADAPT:1;       // 9 Automatic baudrate adjustment control(LIN mode)
    Uint32  MBUFMODE:1;    // 10 Multi-buffer mode
    Uint32  CTYPE:1;       // 11 Checksum type (LIN mode)
    Uint32  HGENCTRL:1;    // 12 Mask filtering comparison control (LIN mode)
    Uint32  STOPEXTFRAME:1;// 13 Stop extended frame communication (LIN mode)
    Uint32  rsvd2:2;       // 15:14  Reserved
    Uint32  LOOPBACK:1;    // 16 Digital loopback mode
    Uint32  CONT:1;        // 17 Continue on suspend
    Uint32  rsvd3:6;       // 23:18  reserved
    Uint32  RXENA:1;       // 24 SCI mode receiver enable
    Uint32  TXENA:1;       // 25 SCI mode transmitter enable
    Uint32  rsvd4:6;       // 31:26  reserved
};

//
// Allow access to the bit fields or entire register
//
union SCIGCR1_REG {
    Uint32               all;
    struct SCIGCR1_BITS  bit;
};

//
// Global Control Register 2 (SCIGCR2) bit definitions
//
struct SCIGCR2_BITS {      // bit    description
    Uint32  POWERDOWN:1;   // 0      Low-power mode PowerDown bit
    Uint32  rsvd1:7;       // 7:1    reserved
    Uint32  GENWU:1;       // 8      Generate Wakeup
    Uint32  rsvd2:7;       // 15:9   reserved
    Uint32  SC:1;          // 16     Send Checksum (LIN mode)
    Uint32  CC:1;          // 17     Compare Checksum (LIN mode)
    Uint32  rsvd3:14;      // 31:18  reserved
};

/* Allow access to the bit fields or entire register */
union SCIGCR2_REG {
    Uint32               all;
    struct SCIGCR2_BITS  bit;
};

//
// SCI Set Interrupt Register (SCISETINT) bit definitions
//
struct SCISETINT_BITS {      // bit  description
    //
    // 0    Set Break-detect Interrupt (SCI compatible mode)
    //
    Uint32  SETBRKDTINT:1;   
    
    Uint32  SETWAKEUPINT:1;  // 1    Set Wake-up Interrupt
    Uint32  rsvd1:2;         // 3:2  reserved
    Uint32  SETTIMEOUTINT:1; // 4    Set Timeout Interrupt (LIN only)
    Uint32  rsvd2:1;         // 5    reserved
    
    //
    // 6    Set Timeout After Wakeup Signal Interrupt (LIN only)
    //
    Uint32  SETTOAWUSINT:1; 

    //
    // 7    Set Timeout After 3 Wakeup Signals Interrupt (LIN only)
    //
    Uint32  SETTOA3WUSINT:1; 
    
    Uint32  SETTXINT:1;      // 8    Set Transmitter Interrupt
    Uint32  SETRXINT:1;      // 9    Receiver Interrupt Enable
    Uint32  rsvd3:3;         // 12:10 reserved
    Uint32  SETIDINT:1;      // 13   Set Identifier Interrupt (LIN only)
    Uint32  rsvd4:2;         // 15:14 reserved
    Uint32  rsvd5:2;         // 17:16 reserved
    Uint32  rsvd6:1;         // 18   reserved
    Uint32  rsvd7:5;         // 23:19 reserved
    Uint32  SETPEINT:1;      // 24   Set Parity Interrupt
    Uint32  SETOEINT:1;      // 25   Set Overrun-Error Interrupt
    Uint32  SETFEINT:1;      // 26   Set Framing-Error Interrupt
    Uint32  SETNREINT:1;     // 27   Set No-Response-Error Interrupt (LIN only)
    
    //
    // 28   Set Inconsistent-Synch-Field-Error Interrupt (LIN only)
    //
    Uint32  SETISFEINT:1;    
    
    Uint32  SETCEINT:1;      //29   Set Checksum-error Interrupt (LIN only)
    Uint32  SETPBEINT:1;     //30   Set Physical Bus Error Interrupt (LIN only)
    Uint32  SETBEINT:1;      //31   Set Bit Error Interrupt (LIN only)
};

//
// Allow access to the bit fields or entire register
//
union SCISETINT_REG {
    Uint32                 all;
    struct SCISETINT_BITS  bit;
};

//
// SCI Clear Interrupt (SCICLEARINT) Register bit definitions
//
struct SCICLEARINT_BITS {    // bit  description
    //
    // 0    Clear Break-detect Interrupt (SCI compatible mode)
    //
    Uint32  CLRBRKDTINT:1;   
    
    Uint32  CLRWAKEUPINT:1;  // 1    Clear Wake-up Interrupt
    Uint32  rsvd1:2;         // 3:2  reserved
    Uint32  CLRTIMEOUTINT:1; // 4    Clear Timeout Interrupt (LIN only)
    Uint32  rsvd2:1;         // 5    reserved
    
    //
    // 6    Clear Timeout After Wakeup Signal Interrupt (LIN only)
    //
    Uint32  CLRTOAWUSINT:1;  
    
    //
    // 7    Clear Timeout After 3 Wakeup Signals Interrupt (LIN only)
    //
    Uint32  CLRTOA3WUSINT:1; 
    
    Uint32  CLRTXINT:1;      // 8    Clear Transmitter Interrupt
    Uint32  CLRRXINT:1;      // 9    Clear Receiver Interrupt
    Uint32  rsvd3:3;         // 12:10 reserved
    Uint32  CLRIDINT:1;      // 13   Clear Identifier Interrupt (LIN only)
    Uint32  rsvd4:2;         // 15:14 reserved
    Uint32  rsvd5:2;         // 17:16 reserved
    Uint32  rsvd6:1;         // 18   reserved
    Uint32  rsvd7:5;         // 23:19 reserved
    Uint32  CLRPEINT:1;      // 24   Clear Parity Interrupt
    Uint32  CLROEINT:1;      // 25   Clear Overrun-Error Interrupt
    Uint32  CLRFEINT:1;      // 26   Clear Framing-Error Interrupt
    
    //
    // 27   Clear No-Response-Error Interrupt (LIN only)
    //
    Uint32  CLRNREINT:1;     
    
    //
    // 28   Clear Inconsistent-Synch-Field-Error Interrupt (LIN only)
    //
    Uint32  CLRISFEINT:1;    
    
    Uint32  CLRCEINT:1;      // 29   Clear Checksum-error Interrupt (LIN only)
    
    //
    // 30   Clear Physical Bus Error Interrupt (LIN only)
    //
    Uint32  CLRPBEINT:1;     
    
    Uint32  CLRBEINT:1;      // 31   Clear Bit Error Interrupt (LIN only)
};

//
// Allow access to the bit fields or entire register
//
union SCICLEARINT_REG {
    Uint32                   all;
    struct SCICLEARINT_BITS  bit;
};

//
// SCI Set Interrupt Level Register (SCISETINTLVL) bit definitions
//
struct SCISETINTLVL_BITS {      // bit  description
    //
    // 0    Set Break-detect Interrupt Level (SCI compatible mode)
    //
    Uint32  SETBRKDTINTLVL:1;   
    
    Uint32  SETWAKEUPINTLVL:1;  // 1    Set Wake-up Interrupt Level
    Uint32  rsvd1:2;            // 3:2  reserved
    Uint32  SETTIMEOUTINTLVL:1; // 4    Set Timeout Interrupt Level (LIN only)
    Uint32  rsvd2:1;            // 5    reserved
    
    //
    // 6    Set Timeout After Wakeup Signal Interrupt Level (LIN only)
    //
    Uint32  SETTOAWUSINTLVL:1;  
    
    //
    // 7    Set Timeout After 3 Wakeup Signals Interrupt Level (LIN only)
    //
    Uint32  SETTOA3WUSINTLVL:1; 
    
    Uint32  SETTXINTLVL:1;    // 8    Set Transmitter Interrupt Level
    Uint32  SETRXINTLVL:1;    // 9    Receiver Interrupt Enable Level
    Uint32  rsvd3:3;          // 12:10  reserved
    Uint32  SETIDINTLVL:1;    // 13   Set Identifier Interrupt Level (LIN only)
    Uint32  rsvd4:2;          // 15:14  reserved
	Uint32  rsvd5:2;          // 17:16  reserved
    Uint32  rsvd6:1;          // 18     reserved
    Uint32  rsvd7:5;          // 23:19  reserved
    Uint32  SETPEINTLVL:1;    // 24    Set Parity Interrupt Level
    Uint32  SETOEINTLVL:1;    // 25    Set Overrun-Error Interrupt Level
    Uint32  SETFEINTLVL:1;    // 26    Set Framing-Error Interrupt Level
    
    //
    // 27    Set No-Response-Error Interrupt Level (LIN only)
    //
    Uint32  SETNREINTLVL:1;   
    
    //
    // 28    Set Inconsistent-Synch-Field-Error Interrupt Level (LIN only)
    //
    Uint32  SETISFEINTLVL:1;  
    
    //
    // 29    Set Checksum-error Interrupt Level (LIN only)
    //
    Uint32  SETCEINTLVL:1;    
    
    //
    // 30    Set Physical Bus Error Interrupt Level (LIN only)
    //
    Uint32  SETPBEINTLVL:1;   
    
    Uint32  SETBEINTLVL:1;    // 31    Set Bit Error Interrupt Level(LIN only)
};

//
// Allow access to the bit fields or entire register
//
union SCISETINTLVL_REG {
    Uint32                    all;
    struct SCISETINTLVL_BITS  bit;
};

//
// SCI Clear Interrupt Level (SCICLEARINTLVL) Register bit definitions
//
struct SCICLEARINTLVL_BITS {    // bit  description
    
    //
    // 0    Clear Break-detect Interrupt Level (SCI compatible mode)
    //
    Uint32  CLRBRKDTINTLVL:1;   
    Uint32  CLRWAKEUPINTLVL:1;  // 1    Clear Wake-up Interrupt Level
    Uint32  rsvd1:2;            // 3:2  reserved
    Uint32  CLRTIMEOUTINTLVL:1; // 4   Clear Timeout Interrupt Level (LIN only)
    Uint32  rsvd2:1;            // 5    reserved
    
    //
    // 6    Clear Timeout After Wakeup Signal Interrupt Level (LIN only)
    //
    Uint32  CLRTOAWUSINTLVL:1;  
    
    //
    // 7    Clear Timeout After 3 Wakeup Signals Interrupt Level (LIN only)
    //
    Uint32  CLRTOA3WUSINTLVL:1; 
    Uint32  CLRTXINTLVL:1;      // 8    Clear Transmitter Interrupt Level
    Uint32  CLRRXINTLVL:1;      // 9    Clear Receiver Interrupt Level
    Uint32  rsvd3:3;            // 12:10 reserved
    
    //
    // 13   Clear Identifier Interrupt Level (LIN only)
    //
    Uint32  CLRIDINTLVL:1;      
    Uint32  rsvd4:2;            // 15:14 reserved
	Uint32  rsvd5:2;            // 17:16 reserved
    Uint32  rsvd6:1;            // 18    reserved
    Uint32  rsvd7:5;            // 23:19 reserved
    Uint32  CLRPEINTLVL:1;      // 24   Clear Parity Interrupt Level
    Uint32  CLROEINTLVL:1;      // 25   Clear Overrun-Error Interrupt Level
    Uint32  CLRFEINTLVL:1;      // 26   Clear Framing-Error Interrupt Level
    
    //
    // 27   Clear No-Response-Error Interrupt Level (LIN only)
    //
    Uint32  CLRNREINTLVL:1;     
    
    //
    // 28   Clear Inconsistent-Synch-Field-Error Interrupt Level (LIN only)
    //
    Uint32  CLRISFEINTLVL:1;    
    
    //
    // 29   Clear Checksum-error Interrupt Level (LIN only)
    //
    Uint32  CLRCEINTLVL:1;      
    
    //
    // 30   Clear Physical Bus Error Interrupt Level (LIN only)
    //
    Uint32  CLRPBEINTLVL:1;
    
    //
    // 31   Clear Bit Error Interrupt Level (LIN only)
    //
    Uint32  CLRBEINTLVL:1;
};

//
// Allow access to the bit fields or entire register
//
union SCICLEARINTLVL_REG {
    Uint32                      all;
    struct SCICLEARINTLVL_BITS  bit;
};

//
// SCI Flags Register (SCIFLR) bit definitions
//
struct SCIFLR_BITS {            // bit description
    Uint32  BRKDT:1;            // 0   Break-detect Flag (SCI compatible mode)
    Uint32  WAKEUP:1;           // 1   Wake-up Flag
    
    //
    // 2   SCI receiver in idle state (SCI compatible mode)
    //
    Uint32  IDLE:1;             
    Uint32  BUSY:1;             // 3   Busy Flag
    Uint32  TIMEOUT:1;          // 4   LIN Bus IDLE timeout Flag (LIN only)
    Uint32  rsvd2:1;            // 5   reserved
    
    //
    // 6   Timeout After Wakeup Signal Flag (LIN only)
    //
    Uint32  TOAWUS:1;           
    
    //
    // 7   Timeout After 3 Wakeup Signals Flag (LIN only)
    //
    Uint32  TOA3WUS:1;          
    Uint32  TXRDY:1;            // 8   Transmitter Buffer Ready Flag
    Uint32  RXRDY:1;            // 9   Receiver Buffer Ready Flag
    Uint32  TXWAKE:1;	        // 10  SCI Transmitter Wakeup Method Select
    Uint32  TXEMPTY:1;          // 11  Transmitter Empty Clag
    Uint32  RXWAKE:1;           // 12  Receiver Wakeup Detect Flag
    Uint32  IDTXFLAG:1;         // 13  Identifier On Transmit Flag (LIN only)
    Uint32  IDRXFLAG:1;         // 14  Identifier on Receive Flag
    Uint32	rsvd3:1;			// 15
    Uint32  rsvd4:8;            // 23:16  reserved
    Uint32  PE:1;               // 24  Parity Error Flag
    Uint32  OE:1;               // 25  Overrun Error Flag
    Uint32  FE:1;               // 26  Framing Error Flag
    Uint32  NRE:1;              // 27  No-Response Error Flag (LIN only)
    
    //
    // 28  Inconsistent Synch Field Error Flag (LIN only)
    //
    Uint32  ISFE:1;             
    Uint32  CE:1;               // 29  Checksum Error Flag (LIN only)
    Uint32  PBE:1;              // 30  Physical Bus Error Flag (LIN only)
    Uint32  BE:1;               // 31  Bit Error Flag (LIN only)
};

//
// Allow access to the bit fields or entire register
//
union SCIFLR_REG {
    Uint32             all;
    struct SCIFLR_BITS  bit;
};

//
// SCI Interrupt Vector Offset 0 (SCIINTVECT0) bit definitions
//
struct SCIINTVECT0_BITS {      // bit    description
    Uint32  INTVECT0:5;        // 4:0    Interrupt vector offset for INT0
    Uint32  rsvd1:11;          // 15:5   reserved
    Uint32  rsvd2:16;          // 31:16  reserved
};

//
// Allow access to the bit fields or entire register
//
union SCIINTVECT0_REG {
    Uint32               all;
    struct SCIINTVECT0_BITS  bit;
};

//
// SCI Interrupt Vector Offset 1 (SCIINTVECT1) bit definitions
//
struct SCIINTVECT1_BITS {      // bit    description
    Uint32  INTVECT1:5;        // 4:0    Interrupt vector offset for INT1
    Uint32  rsvd1:11;          // 15:5   reserved
    Uint32  rsvd2:16;          // 31:16  reserved
};

//
// Allow access to the bit fields or entire register
//
union SCIINTVECT1_REG {
    Uint32               all;
    struct SCIINTVECT1_BITS  bit;
};

//
// SCI Format Control Register (SCIFORMAT) bit definitions
//
struct SCIFORMAT_BITS {        // bit    description
    Uint32  CHAR:3;            // 2:0    Character Length Control Bits
    Uint32  rsvd1:13;          // 15:3   reserved
    Uint32  LENGTH:3;          // 18:16  Frame Length Control Bits
    Uint32  rsvd2:13;          // 31:19  reserved
};

//
// Allow access to the bit fields or entire register
//
union SCIFORMAT_REG {
    Uint32                 all;
    struct SCIFORMAT_BITS  bit;
};

//
// Baud Rate Selection Register (BRSR) bit definitions
//
struct BRSR_BITS {             // bit    description
	Uint32  SCI_LIN_PSL :16;   // 15:0   SCI/LIN Prescaler Low
	Uint32  SCI_LIN_PSH :8;    // 23:16  SCI/LIN Prescaler High
    Uint32  M:4;               // 27:24  SCI/LIN Fractional Divider Selection
    Uint32  rsvd1:4;           // 31:28     reserved
};

//
// Allow access to the bit fields or entire register
//
union BRSR_REG {
    Uint32               all;
    struct BRSR_BITS     bit;
};

//
// SCI Pin I/O Control Register 2 (SCIPIO2) bit definitions
//
struct SCIPIO2_BITS {          // bit    description
    Uint32  rsvd1:1;           // 0      reserved
    Uint32  RXIN:1;            // 1      SCIRX pin value
    Uint32  TXIN:1;            // 2      SCITX pin value
    Uint32  rsvd2:13;          // 15:3   reserved
	Uint32  rsvd3:16;          // 31:16  reserved
};

//
// Allow access to the bit fields or entire register
//
union SCIPIO2_REG {
    Uint32               all;
    struct SCIPIO2_BITS  bit;
};

//
// LIN Compare Register (LINCOMP) bit definitions
//
struct LINCOMP_BITS {          // bit    description
    Uint32  SBREAK:3;          // 2:0    Synch Break Extend
    Uint32  rsvd1:5;           // 7:3    reserved
    Uint32  SDEL:2;            // 9:8    Sync Delimiter Compare
	Uint32  rsvd2:6;           // 15:10  reserved
	Uint32  rsvd3:16;          // 31:16  reserved
};

//
// Allow access to the bit fields or entire register
//
union LINCOMP_REG {
    Uint32               all;
    struct LINCOMP_BITS  bit;
};

//
// LIN Receive Data 0 Register (LINRD0) bit definitions
//
struct LINRD0_BITS {           // bit    description
    Uint32  RD3:8;             // 7:0    Receive Buffer 3
    Uint32  RD2:8;             // 15:8   Receive Buffer 2
    Uint32  RD1:8;             // 23:16  Receive Buffer 1
    Uint32  RD0:8;             // 31:24  Receive Buffer 0
};

//
// Allow access to the bit fields or entire register
//
union LINRD0_REG {
    Uint32              all;
    struct LINRD0_BITS  bit;
};

//
// LIN Receive Data 1 Register (LINRD1) bit definitions
//
struct LINRD1_BITS {           // bit    description
    Uint32  RD7:8;             // 7:0    Receive Buffer 7
    Uint32  RD6:8;             // 15:8   Receive Buffer 6
    Uint32  RD5:8;             // 23:16  Receive Buffer 5
    Uint32  RD4:8;             // 31:24  Receive Buffer 4
};

//
// Allow access to the bit fields or entire register
//
union LINRD1_REG {
    Uint32              all;
    struct LINRD1_BITS  bit;
};

//
// LIN Acceptance Mask Register (LINMASK) bit definitions
//
struct LINMASK_BITS {          // bit    description
    Uint32  TXIDMASK:8;        // 7:0    TX ID Mask bits (LIN only)
    Uint32  rsvd1:8;           // 15:8   reserved
    Uint32  RXIDMASK:8;        // 23:16  RX ID Mask bits (LIN only)
    Uint32  rsvd2:8;           // 31:24  reserved
};

//
// Allow access to the bit fields or entire register
//
union LINMASK_REG {
    Uint32               all;
    struct LINMASK_BITS  bit;
};

//
// LIN ID Register (LINID) bit definitions
//
struct LINID_BITS {            // bit    description
    Uint32  IDBYTE:8;          // 7:0    LIN message ID (LIN only)
    Uint32  IDSLAVETASKBYTE:8; // 15:8   Received ID comparison ID (LIN only)
    Uint32  RECEIVEDID:8;      // 23:16  Current Message ID (LIN only)
    Uint32  rsvd1:8;           // 31:24  reserved
};

//
// Allow access to the bit fields or entire register
//
union LINID_REG {
    Uint32             all;
    struct LINID_BITS  bit;
};

//
// LIN Transmit Data 0 Register (LINTD0) bit definitions
//
struct LINTD0_BITS {           // bit    description
    Uint32  TD3:8;             // 7:0    Transmit Buffer 3
    Uint32  TD2:8;             // 15:8   Transmit Buffer 2
    Uint32  TD1:8;             // 23:16  Transmit Buffer 1
    Uint32  TD0:8;             // 31:24  Transmit Buffer 0
};

//
// Allow access to the bit fields or entire register
//
union LINTD0_REG {
    Uint32              all;
    struct LINTD0_BITS  bit;
};

//
// LIN Transmit Data 1 Register (LINTD1) bit definitions
//
struct LINTD1_BITS {           // bit    description
    Uint32  TD7:8;             // 7:0    Transmit Buffer 7
    Uint32  TD6:8;             // 15:8   Transmit Buffer 6
    Uint32  TD5:8;             // 23:16  Transmit Buffer 5
    Uint32  TD4:8;             // 31:24  Transmit Buffer 4
};

//
// Allow access to the bit fields or entire register
//
union LINTD1_REG {
    Uint32               all;
    struct LINTD1_BITS   bit;
};

//
// IODFT for LIN (IODFTCTRL) bit definitions
//
struct IODFTCTRL_BITS {        // bit    description
    Uint32  RXPENA:1;          // 0      Analog Loopback Via Receive Pin Enable
    Uint32  LPBENA:1;          // 1      Module Loopback Enable
    Uint32  rsvd1:6;           // 7:2    reserved
    Uint32  IODFTENA:4;        // 11:8   IO DFT Enable Key
    Uint32  rsvd2:4;           // 15:12  Reserved
    Uint32  TXSHIFT:3;         // 18:16  Transmit Delay Shift
    Uint32  PINSAMPLEMASK:2;   // 20:19  TX Pin Sample Mask
    Uint32  rsvd3:3;           // 23:21  Reserved
    
    //
    // 24     Break Detect Error Enable (SCI compatibility mode)
    //
    Uint32  BRKDTERRENA:1;     
    
    //
    // 25     Parity Error Enable (SCI compatibility mode)
    //
    Uint32  PERRENA:1;         
    
    //
    // 26     Frame Error Enable (SCI compatibility mode)
    //
    Uint32  FERRENA:1;         
    
    Uint32  rsvd:1;            // 27     reserved
    
    //
    // 28     Inconsistent Synch Field Error Enable (LIN mode)
    //
    Uint32  ISFERRENA:1;       
    
    Uint32  CERRENA:1;         // 29     Checksum Error Enable(LIN mode)
    Uint32  PBERRENA:1;        // 30     Physical Bus Error Enable (LIN mode)
    Uint32  BERRENA:1;         // 31     Bit Error Enable (LIN mode)
};

//
// Allow access to the bit fields or entire register
//
union IODFTCTRL_REG {
    Uint32                 all;
    struct IODFTCTRL_BITS  bit;
};

//
// LIN register file
//
struct LIN_REGS {
    union   SCIGCR0_REG         SCIGCR0;         // Global Control Register 0		0
	union	SCIGCR1_REG		    SCIGCR1;		 // Global Control Register 1		4
	union	SCIGCR2_REG		    SCIGCR2;		 // Global Control Register 2		8
	union	SCISETINT_REG		SCISETINT;		 // Interrupt Enable Register		c
	union	SCICLEARINT_REG		SCICLEARINT;	 // Interrupt Disable Register		10
	
    //
    // Set Interrupt Level Register
    //
    union	SCISETINTLVL_REG	SCISETINTLVL;	 				//		14
	
    //
    // Clear Interrupt Level Register
    //
    union	SCICLEARINTLVL_REG	SCICLEARINTLVL;	 			//			18
	union	SCIFLR_REG		    SCIFLR;		     // Flag Register		1c
	
    //
    // Interrupt Vector Offset Register 0
    //
    union	SCIINTVECT0_REG		SCIINTVECT0;	 //				20
	
    //
    // Interrupt Vector Offset Register 1
    //
    union	SCIINTVECT1_REG	 	SCIINTVECT1;	 //				24
	union	SCIFORMAT_REG		SCIFORMAT;	    //Length Control Register			28
	union	BRSR_REG			BRSR;		    //Baud Rate Selection Register		2c
	Uint32          			SCIED;			//Emulation buffer Register			30
	Uint32          			SCIRD;			//Receiver data buffer Register		3c
	Uint32          			SCITD;			//Transmit data buffer Register		40
	union	SCIPIO2_REG			SCIPIO2;		//Pin control Register 2			44
	union	LINCOMP_REG			LINCOMP;		//Compare register					48
	union	LINRD0_REG			LINRD0;			//Receive data register 0			4c
	union	LINRD1_REG			LINRD1;			//Receive data register 1			50
	union	LINMASK_REG			LINMASK;		//Acceptance mask register			54
	union	LINID_REG			LINID;			//LIN ID Register					58
	union	LINTD0_REG			LINTD0;			//Transmit Data Register 0			5c
	union	LINTD1_REG			LINTD1;			//Transmit Data Register 1			60
	Uint32			            MBRSR;			//Baud Rate Selection Register		64
	union   IODFTCTRL_REG		IODFTCTRL;		//IODFT for LIN						68
};

//
// LIN External References & Function Declarations:
//
extern volatile struct LIN_REGS *const  P_LinaRegs;
extern volatile struct LIN_REGS LinaRegs;


#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  

//
// End of file
//

