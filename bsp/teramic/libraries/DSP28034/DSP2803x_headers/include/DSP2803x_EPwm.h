#ifndef DSP2803x_EPWM_H
#define DSP2803x_EPWM_H


#ifdef __cplusplus
extern "C" {
#endif

//----------------------------------------------------
// Time base control register bit definitions */
struct TBCTL_BITS {          // bits   description
   Uint32 CTRMODE:2;         // 1:0    Counter Mode
   Uint32 PHSEN:1;           // 2      Phase load enable
   Uint32 PRDLD:1;           // 3      Active period load
   Uint32 SYNCOSEL:2;        // 5:4    Sync output select
   Uint32 SWFSYNC:1;         // 6      Software force sync pulse
   Uint32 HSPCLKDIV:3;       // 9:7    High speed time pre-scale
   Uint32 CLKDIV:3;          // 12:10  Timebase clock pre-scale
   Uint32 PHSDIR:1;          // 13     Phase Direction
   Uint32 FREE_SOFT:2;       // 15:14  Emulation mode
   Uint32 rsvd:16;               //31:16 reserved
};

union TBCTL_REG {
   Uint32              all;
   struct TBCTL_BITS   bit;
};

//----------------------------------------------------
// Time base status register bit definitions */
struct TBSTS_BITS {          // bits   description
   Uint32 CTRDIR:1;          // 0      Counter direction status
   Uint32 SYNCI:1;           // 1      External input sync status
   Uint32 CTRMAX:1;          // 2      Counter max latched status
   Uint32 rsvd1:13;          // 15:3   reserved
   Uint32 rsvd2:16;           //31:16 reserved
};

union TBSTS_REG {
   Uint32              all;
   struct TBSTS_BITS   bit;
};

//----------------------------------------------------
// Compare control register bit definitions */
struct CMPCTL_BITS {          // bits   description
   Uint32 LOADAMODE:2;        // 0:1    Active compare A
   Uint32 LOADBMODE:2;        // 3:2    Active compare B
   Uint32 SHDWAMODE:1;        // 4      Compare A block operating mode
   Uint32 rsvd1:1;            // 5      reserved
   Uint32 SHDWBMODE:1;        // 6      Compare B block operating mode
   Uint32 rsvd2:1;            // 7      reserved
   Uint32 SHDWAFULL:1;        // 8      Compare A Shadow registers full Status
   Uint32 SHDWBFULL:1;        // 9      Compare B Shadow registers full Status
   Uint32 rsvd3:6;            // 15:10  reserved
   Uint32 rsvd4:16;          //31:16 reserved
};


union CMPCTL_REG {
   Uint32                all;
   struct CMPCTL_BITS    bit;
};

//----------------------------------------------------
// Action qualifier register bit definitions */
struct AQCTL_BITS {           // bits   description
   Uint32 ZRO:2;              // 1:0    Action Counter = Zero
   Uint32 PRD:2;              // 3:2    Action Counter = Period
   Uint32 CAU:2;              // 5:4    Action Counter = Compare A up
   Uint32 CAD:2;              // 7:6    Action Counter = Compare A down
   Uint32 CBU:2;              // 9:8    Action Counter = Compare B up
   Uint32 CBD:2;              // 11:10  Action Counter = Compare B down
   Uint32 rsvd:4;             // 15:12  reserved
   Uint32 rsvd1:16;        //31:16 reserved
};

union AQCTL_REG {
   Uint32                all;
   struct AQCTL_BITS     bit;
};

//----------------------------------------------------
// Action qualifier SW force register bit definitions */
struct AQSFRC_BITS {           // bits   description
   Uint32 ACTSFA:2;            // 1:0    Action when One-time SW Force A invoked
   Uint32 OTSFA:1;             // 2      One-time SW Force A output
   Uint32 ACTSFB:2;            // 4:3    Action when One-time SW Force B invoked
   Uint32 OTSFB:1;             // 5      One-time SW Force A output
   Uint32 RLDCSF:2;            // 7:6    Reload from Shadow options
   Uint32 rsvd1:8;             // 15:8   reserved
   Uint32 rsvd2:16;          //31:16 reserved
};

union AQSFRC_REG {
   Uint32                 all;
   struct AQSFRC_BITS     bit;
};

//----------------------------------------------------
// Action qualifier continuous SW force register bit definitions */
struct AQCSFRC_BITS {          // bits   description
   Uint32 CSFA:2;              // 1:0    Continuous Software Force on output A
   Uint32 CSFB:2;              // 3:2    Continuous Software Force on output B
   Uint32 rsvd1:12;            // 15:4   reserved
   Uint32 rsvd2:16;           //31:16 reserved
};

union AQCSFRC_REG {
   Uint32                  all;
   struct AQCSFRC_BITS     bit;
};


//----------------------------------------------------
// Dead-band generator control register bit definitions
struct DBCTL_BITS {          // bits   description
   Uint32 OUT_MODE:2;      	 // 1:0    Dead Band Output Mode Control
   Uint32 POLSEL:2;          // 3:2    Polarity Select Control
   Uint32 IN_MODE:2;         // 5:4    Dead Band Input Select Mode Control
   Uint32 rsvd1:8;           // 13:6   reserved
   Uint32 HALFEN:1;         //14     Judge whether there is HALFCYCLE
   Uint32 HALFCYCLE:1;       // 15     Half Cycle Clocking Enable
   Uint32 rsvd2:16;             //31:16 reserved
};

union DBCTL_REG {
   Uint32                  all;
   struct DBCTL_BITS       bit;
};


//----------------------------------------------------
// Trip zone select register bit definitions
struct TZSEL_BITS {           // bits   description
   Uint32  CBC1:1;            // 0      TZ1 CBC select
   Uint32  CBC2:1;            // 1      TZ2 CBC select
   Uint32  CBC3:1;            // 2      TZ3 CBC select
   Uint32  CBC4:1;            // 3      TZ4 CBC select
   Uint32  CBC5:1;            // 4      TZ5 CBC select
   Uint32  CBC6:1;            // 5      TZ6 CBC select
   Uint32  DCAEVT2:1;         // 6      DCAEVT2
   Uint32  DCBEVT2:1;         // 7      DCBEVT2
   Uint32  OSHT1:1;           // 8      One-shot TZ1 select
   Uint32  OSHT2:1;           // 9      One-shot TZ2 select
   Uint32  OSHT3:1;           // 10     One-shot TZ3 select
   Uint32  OSHT4:1;           // 11     One-shot TZ4 select
   Uint32  OSHT5:1;           // 12     One-shot TZ5 select
   Uint32  OSHT6:1;           // 13     One-shot TZ6 select
   Uint32  DCAEVT1:1;         // 14     DCAEVT1
   Uint32  DCBEVT1:1;         // 15     DCBEVT1
   Uint32  rsvd:16;               //31:16 reserved
};

union TZSEL_REG {
   Uint32                  all;
   struct TZSEL_BITS       bit;
};


//----------------------------------------------------
// Trip zone digital compare event select register
struct TZDCSEL_BITS {        // bits   description
   Uint32  DCAEVT1:3;        // 2:0    Digital Compare Output A Event 1
   Uint32  DCAEVT2:3;        // 5:3    Digital Compare Output A Event 2
   Uint32  DCBEVT1:3;        // 8:6    Digital Compare Output B Event 1
   Uint32  DCBEVT2:3;        // 11:9   Digital Compare Output B Event 2
   Uint32  rsvd:20;             //31:12   reserved
};

union TZDCSEL_REG {
   Uint32                  all;
   struct TZDCSEL_BITS     bit;
};

//----------------------------------------------------
// Trip zone control register bit definitions */
struct TZCTL_BITS {         // bits   description
   Uint32 TZA:2;            // 1:0    TZ1 to TZ6 Trip Action On EPWMxA
   Uint32 TZB:2;            // 3:2    TZ1 to TZ6 Trip Action On EPWMxB
   Uint32 DCAEVT1:2;        // 5:4    EPWMxA action on DCAEVT1
   Uint32 DCAEVT2:2;        // 7:6    EPWMxA action on DCAEVT2
   Uint32 DCBEVT1:2;        // 9:8    EPWMxB action on DCBEVT1
   Uint32 DCBEVT2:2;        // 11:10  EPWMxB action on DCBEVT2
   Uint32 rsvd:4;           // 15:12  reserved
   Uint32 rsvd1:16;         //31:16 reserved
};

union TZCTL_REG {
   Uint32                  all;
   struct TZCTL_BITS       bit;
};


//----------------------------------------------------
// Trip zone control register bit definitions */
struct TZEINT_BITS {         // bits   description
   Uint32  rsvd1:1;          // 0      reserved
   Uint32  CBC:1;            // 1      Trip Zones Cycle By Cycle Int Enable
   Uint32  OST:1;            // 2      Trip Zones One Shot Int Enable
   Uint32  DCAEVT1:1;        // 3      Force DCAEVT1 event
   Uint32  DCAEVT2:1;        // 4      Force DCAEVT2 event
   Uint32  DCBEVT1:1;        // 5      Force DCBEVT1 event
   Uint32  DCBEVT2:1;        // 6      Force DCBEVT2 event
   Uint32  rsvd2:9;          // 15:7   reserved
   Uint32  rsvd3:16;         //31:16 reserved
};


union TZEINT_REG {
   Uint32                  all;
   struct TZEINT_BITS      bit;
};


//----------------------------------------------------
// Trip zone flag register bit definitions */
struct TZFLG_BITS {         // bits   description
   Uint32  INT:1;           // 0      Global status
   Uint32  CBC:1;           // 1      Trip Zones Cycle By Cycle Int
   Uint32  OST:1;           // 2      Trip Zones One Shot Int
   Uint32  DCAEVT1:1;       // 3      Force DCAEVT1 event
   Uint32  DCAEVT2:1;       // 4      Force DCAEVT2 event
   Uint32  DCBEVT1:1;       // 5      Force DCBEVT1 event
   Uint32  DCBEVT2:1;       // 6      Force DCBEVT2 event
   Uint32  rsvd2:9;         // 15:7   reserved
   Uint32  rsvd3:16;        //31:16 reserved
};

union TZFLG_REG {
   Uint32                  all;
   struct TZFLG_BITS       bit;
};

//----------------------------------------------------
// Trip zone flag clear register bit definitions */
struct TZCLR_BITS {         // bits   description
   Uint32  INT:1;           // 0      Global status
   Uint32  CBC:1;           // 1      Trip Zones Cycle By Cycle Int
   Uint32  OST:1;           // 2      Trip Zones One Shot Int
   Uint32  DCAEVT1:1;       // 3      Force DCAEVT1 event
   Uint32  DCAEVT2:1;       // 4      Force DCAEVT2 event
   Uint32  DCBEVT1:1;       // 5      Force DCBEVT1 event
   Uint32  DCBEVT2:1;       // 6      Force DCBEVT2 event
   Uint32  rsvd2:9;         // 15:7   reserved
   Uint32  rsvd3:16;          //31:16 reserved
};

union TZCLR_REG {
   Uint32                  all;
   struct TZCLR_BITS       bit;
};

//----------------------------------------------------
// Trip zone flag force register bit definitions */
struct TZFRC_BITS {         // bits   description
   Uint32  rsvd1:1;         // 0      reserved
   Uint32  CBC:1;           // 1      Trip Zones Cycle By Cycle Int
   Uint32  OST:1;           // 2      Trip Zones One Shot Int
   Uint32  DCAEVT1:1;       // 3      Force DCAEVT1 event
   Uint32  DCAEVT2:1;       // 4      Force DCAEVT2 event
   Uint32  DCBEVT1:1;       // 5      Force DCBEVT1 event
   Uint32  DCBEVT2:1;       // 6      Force DCBEVT2 event
   Uint32  rsvd2:9;        // 15:7   reserved
   Uint32  rsvd3:16;        //31:16 reserved
};

union TZFRC_REG {
   Uint32                  all;
   struct TZFRC_BITS       bit;
};

//----------------------------------------------------
// Event trigger select register bit definitions */
struct ETSEL_BITS {         // bits   description
   Uint32  INTSEL:3;        // 2:0    EPWMxINTn Select
   Uint32  INTEN:1;         // 3      EPWMxINTn Enable
   Uint32  rsvd1:4;         // 7:4    reserved
   Uint32  SOCASEL:3;       // 10:8   Start of conversion A Select
   Uint32  SOCAEN:1;        // 11     Start of conversion A Enable
   Uint32  SOCBSEL:3;       // 14:12  Start of conversion B Select
   Uint32  SOCBEN:1;        // 15     Start of conversion B Enable
   Uint32  rsvd2:16;                 //31:16 reserved
};

union ETSEL_REG {
   Uint32                  all;
   struct ETSEL_BITS       bit;
};


//----------------------------------------------------
// Event trigger pre-scale register bit definitions */
struct ETPS_BITS {         // bits   description
   Uint32  INTPRD:2;       // 1:0    EPWMxINTn Period Select
   Uint32  INTCNT:2;       // 3:2    EPWMxINTn Counter Register
   Uint32  rsvd1:4;        // 7:4    reserved
   Uint32  SOCAPRD:2;      // 9:8    EPWMxSOCA Period Select
   Uint32  SOCACNT:2;      // 11:10  EPWMxSOCA Counter Register
   Uint32  SOCBPRD:2;      // 13:12  EPWMxSOCB Period Select
   Uint32  SOCBCNT:2;      // 15:14  EPWMxSOCB Counter Register
   Uint32  rsvd2:16;              //31:16 reserved
};

union ETPS_REG {
   Uint32                  all;
   struct ETPS_BITS        bit;
};

//----------------------------------------------------
// Event trigger Flag register bit definitions */
struct ETFLG_BITS {         // bits   description
   Uint32  INT:1;           // 0	  EPWMxINTn Flag
   Uint32  rsvd1:1;         // 1	  reserved
   Uint32  SOCA:1;          // 2	  EPWMxSOCA Flag
   Uint32  SOCB:1;          // 3	  EPWMxSOCB Flag
   Uint32  rsvd2:12;        // 15:4	  reserved
   Uint32  rsvd3:16;         //31:16 reserved
};

union ETFLG_REG {
   Uint32                   all;
   struct ETFLG_BITS        bit;
};


//----------------------------------------------------
// Event trigger Clear register bit definitions */
struct ETCLR_BITS {         // bits   description
   Uint32  INT:1;           // 0	  EPWMxINTn Clear
   Uint32  rsvd1:1;         // 1	  reserved
   Uint32  SOCA:1;          // 2	  EPWMxSOCA Clear
   Uint32  SOCB:1;          // 3	  EPWMxSOCB Clear
   Uint32  rsvd2:12;        // 15:4	  reserved
   Uint32  rsvd3:16;      //31:16 reserved
};

union ETCLR_REG {
   Uint32                   all;
   struct ETCLR_BITS        bit;
};

//----------------------------------------------------
// Event trigger Force register bit definitions */
struct ETFRC_BITS {         // bits   description
   Uint32  INT:1;           // 0	  EPWMxINTn Force
   Uint32  rsvd1:1;         // 1	  reserved
   Uint32  SOCA:1;          // 2	  EPWMxSOCA Force
   Uint32  SOCB:1;          // 3	  EPWMxSOCB Force
   Uint32  rsvd2:12;        // 15:4	  reserved
   Uint32  rsvd3:16;         //31:16 reserved
};

union ETFRC_REG {
   Uint32                  all;
   struct ETFRC_BITS        bit;
};
//----------------------------------------------------
// PWM chopper control register bit definitions */
struct PCCTL_BITS {         // bits   description
   Uint32  CHPEN:1;         // 0      PWM chopping enable
   Uint32  OSHTWTH:4;       // 4:1    One-shot pulse width
   Uint32  CHPFREQ:3;       // 7:5    Chopping clock frequency
   Uint32  CHPDUTY:3;       // 10:8   Chopping clock Duty cycle
   Uint32  rsvd1:5;         // 15:11  reserved
   Uint32  rsvd2:16;          //31:16 reserved
};


union PCCTL_REG {
   Uint32                  all;
   struct PCCTL_BITS       bit;
};


//----------------------------------------------------
// Enhanced Trip register bit definitions */

struct DCTRIPSEL_BITS {      // bits   description
   Uint32 DCAHCOMPSEL:4;     // 3:0    Digital Compare A High, COMP Input Select
   Uint32 DCALCOMPSEL:4;     // 7:4    Digital Compare A Low, COMP Input Select
   Uint32 DCBHCOMPSEL:4;     // 11:8   Digital Compare B High, COMP Input Select
   Uint32 DCBLCOMPSEL:4;     // 15:12  Digital Compare B Low, COMP Input Select
   Uint32 rsvd:16;                    //31:16 reserved
};

union DCTRIPSEL_REG {
   Uint32                  all;
   struct DCTRIPSEL_BITS   bit;
};

struct DCCTL_BITS {          // bits   description
   Uint32 EVT1SRCSEL:1;      // 0      DCBEVT1 Source Signal Select
   Uint32 EVT1FRCSYNCSEL:1;  // 1      DCBEVT1 Force Cynchronization Signal Select
   Uint32 EVT1SOCE:1;        // 2      DCEVT1 SOC, Enable/Disable
   Uint32 EVT1SYNCE:1;       // 3      DCEVT1 Sync, Enable/Disable
   Uint32 rsvd1:4;           // 7:4    reserved
   Uint32 EVT2SRCSEL:1;      // 8      DCEVT2 Source Signal Select
   Uint32 EVT2FRCSYNCSEL:1;  // 9      DCEVT2 Force Synchronization Signal Select
   Uint32 rsvd2:6;           // 15:10  reserved
   Uint32 rsvd3:16;            //31:16 reserved
};

union DCCTL_REG {
   Uint32                  all;
   struct DCCTL_BITS       bit;
};

struct DCCAPCTL_BITS {       // bits   description
   Uint32 CAPE:1;            // 0      Counter Capture Enable/Disable
   Uint32 SHDWMODE:1;        // 1      Counter Capture Mode
   Uint32 rsvd:14;           // 15:2   reserved
   Uint32 rsvd1:16;          //31:16 reserved
};

union DCCAPCTL_REG {
   Uint32                  all;
   struct DCCAPCTL_BITS    bit;
};

struct DCFCTL_BITS {         // bits   description
   Uint32 SRCSEL:2;          // 1:0    Filter Block Signal Source Select
   Uint32 BLANKE:1;          // 2      Blanking Enable/Disable
   Uint32 BLANKINV:1;        // 3      Blanking Window Inversion
   Uint32 PULSESEL:2;        // 5:4    Pulse Select for Blanking & Capture Alignment
   Uint32 rsvd:10;           // 15:6   reserved
   Uint32 rsvd1:16;          //31:16 reserved
};

union DCFCTL_REG {
   Uint32                  all;
   struct DCFCTL_BITS      bit;
};


struct CLKDIV_BITS{
	Uint32      CLKDIV:1;
	Uint32      rsvd:31;
};


union CLKDIV_REG{
	Uint32       all;
	struct  CLKDIV_BITS  bit;
};


//----------------------------------------------------
// High resolution period control register bit definitions */

struct HRPCTL_BITS {        // bits   description
   Uint32  HRPE:1;          // 0      High resolution period enable
   Uint32  PWMSYNCSEL:1;         // 1      PWMSYNC Source Select Bit
   Uint32  TBPHSHRLOADE:1;  // 2      TBPHSHR Load Enable Bit
   Uint32  rsvd2:13;        // 15:3   reserved
   Uint32  rsvd3:16;         //31:16 reserved
};

union HRPCTL_REG {
   Uint32                  	all;
   struct HRPCTL_BITS       bit;
};

//----------------------------------------------------
// High Resolution Register bit definitions */

struct HRCNFG_BITS {       	// bits   description
   Uint32  EDGMODE:2;     	// 1:0    Edge Mode select Bits
   Uint32  CTLMODE:1;     	// 2      Control mode Select Bit
   Uint32  HRLOAD:2;      	// 4:3    Shadow mode Select Bit
   Uint32  SELOUTB:1;                // 5      EPWMB Output Select Bit
   Uint32  AUTOCONV:1;           // 6      Autoconversion Bit
   Uint32  SWAPAB:1;                // 7	  Swap EPWMA & EPWMB Outputs Bit
   Uint32  rsvd1:8;      	// 15:8   reserved
   Uint32  rsvd2:16;                    //31:16 reserved
};

union HRCNFG_REG {
   Uint32                  	all;
   struct HRCNFG_BITS       bit;
};


struct HRPWR_BITS {       	           // bits   description
   Uint32  PHASE_DETECT_SEL:3;          //2:0
   Uint32  DLL_START_POINT_0_2:3;      //5:3
   Uint32  MEPOFF:4;     	          // 9:6    MEP Calibration Off Bits
   Uint32  DLL_START_POINT_3_7:5;     // 14:10
   Uint32  rsvd1:17;                             //31:15
};


union HRPWR_REG {
   Uint32                  	all;
   struct HRPWR_BITS        bit;
};

struct TBPHS_HRPWM_REG {   	// bits   description
   Uint32  TBPHSHR;     	// 31:0   Extension register for HRPWM Phase (8 bits)
   Uint32  TBPHS;                      // 63:32  Phase offset register
};


union TBPHS_HRPWM_GROUP {
   Uint64                  all;
   struct TBPHS_HRPWM_REG  half;
};

struct CMPA_HRPWM_REG {   	// bits   description
   Uint32  CMPAHR;     	    // 31:0   Extension register for HRPWM compare (8 bits)
   Uint32  CMPA;                         // 63:32  Compare A reg
};

union CMPA_HRPWM_GROUP {
   Uint64                 all;
   struct CMPA_HRPWM_REG  half;
};

struct TBPRD_HRPWM_REG {   	// bits   description
   Uint32  TBPRDHR;     	// 31:0   Extension register for HRPWM Period (8 bits)
   Uint32  TBPRD;           // 63:32  Timebase Period Register
};

union TBPRD_HRPWM_GROUP {
   Uint64                  all;
   struct TBPRD_HRPWM_REG  half;
};

struct EPWM_REGS {
   union  TBCTL_REG           TBCTL;       // Time Base Control Register   00
   union  TBSTS_REG           TBSTS;       // Time Base Status Register    04

   union TBPHS_HRPWM_GROUP 	TBPHS;       // 08

   Uint32                     TBCTR;       // Time Base Counter				10
   Uint32                     TBPRD;       // Time Base Period register set  14
   Uint32                     TBPRDHR;     // Time Base Period High Res Register  18

   union  CMPCTL_REG          CMPCTL;      // Compare control   1c

   union  CMPA_HRPWM_GROUP    CMPA;  // 20
   
   Uint32                     CMPB;        // Compare B reg				28

   union  AQCTL_REG           AQCTLA;      // Action qual output A    2C
   union  AQCTL_REG           AQCTLB;      // Action qual output B   30
   union  AQSFRC_REG          AQSFRC;      // Action qual SW force			34
   union  AQCSFRC_REG         AQCSFRC;     // Action qualifier continuous SW force		38


   union  DBCTL_REG           DBCTL;       // Dead-band control					3C
   Uint32                     DBRED;       // Dead-band rising edge delay		40
   Uint32                     DBFED;       // Dead-band falling edge delay      44

   union  TZSEL_REG           TZSEL;       // Trip zone select					48
   union  TZDCSEL_REG         TZDCSEL;     // Trip zone digital comparator select		4C
   union  TZCTL_REG           TZCTL;       // Trip zone control				50
   union  TZEINT_REG          TZEINT;      // Trip zone interrupt enable		54
   union  TZFLG_REG           TZFLG;       // Trip zone interrupt flags			58
   union  TZCLR_REG           TZCLR;       // Trip zone clear				5C
   union  TZFRC_REG    	      TZFRC;       // Trip zone force interrupt  		60

   union  ETSEL_REG           ETSEL;       // Event trigger selection			64
   union  ETPS_REG            ETPS;        // Event trigger pre-scaler			68
   union  ETFLG_REG           ETFLG;       // Event trigger flags				6C
   union  ETCLR_REG           ETCLR;       // Event trigger clear				70
   union  ETFRC_REG           ETFRC;       // Event trigger force				74


   union  PCCTL_REG           PCCTL;       // PWM chopper control				78
   Uint32 rsvd5;
   union  HRCNFG_REG          HRCNFG;      // HRPWM Config Reg				80
   union  HRPWR_REG			  HRPWR;       // HRPWM Power Register				84

   Uint32	rsvd4[4];
   Uint32                     HRMSTEP;     // HRPWM MEP Step Register			98
   Uint32  rsvd2;
   union  HRPCTL_REG          HRPCTL;      // High Resolution Period Control	A0
   union  CLKDIV_REG          CLKDIV;  

   union  TBPRD_HRPWM_GROUP   TBPRDM;	   // Union of TBPRD:TBPRDHR mirror registers  A8
   union  CMPA_HRPWM_GROUP	  CMPAM; 	  // Union of CMPA:CMPAHR mirror registers  B0

   Uint32 rsvd3[2];

   union  DCTRIPSEL_REG       DCTRIPSEL;   // Digital Compare Trip Select       C0
   union  DCCTL_REG           DCACTL;      // Digital Compare A Control			C4
   union  DCCTL_REG           DCBCTL;      // Digital Compare B Control			C8
   union  DCFCTL_REG          DCFCTL;      // Digital Compare Filter Control		CC
   union  DCCAPCTL_REG        DCCAPCTL;    // Digital Compare Capture Control		D0
   Uint32                     DCFOFFSET;   // Digital Compare Filter Offset				D4
   Uint32                     DCFOFFSETCNT;// Digital Compare Filter Offset Counter		D8
   Uint32                     DCFWINDOW;   // Digital Compare Filter Window					DC
   Uint32                     DCFWINDOWCNT;// Digital Compare Filter Window Counter			E0
   Uint32                     DCCAP;       // Digital Compare Filter Counter Capture		E4
};



//---------------------------------------------------------------------------
// External References & Function Declarations:
//



extern volatile struct EPWM_REGS *const P_EPwm1Regs;
extern volatile struct EPWM_REGS EPwm1Regs;


extern volatile struct EPWM_REGS *const P_EPwm2Regs;
extern volatile struct EPWM_REGS EPwm2Regs;


extern volatile struct EPWM_REGS *const P_EPwm3Regs;
extern volatile struct EPWM_REGS EPwm3Regs;


extern volatile struct EPWM_REGS *const P_EPwm4Regs;
extern volatile struct EPWM_REGS EPwm4Regs;

extern volatile struct EPWM_REGS *const P_EPwm5Regs;
extern volatile struct EPWM_REGS EPwm5Regs;

extern volatile struct EPWM_REGS *const P_EPwm6Regs;
extern volatile struct EPWM_REGS EPwm6Regs;

extern volatile struct EPWM_REGS *const P_EPwm7Regs;
extern volatile struct EPWM_REGS EPwm7Regs;






#ifdef __cplusplus
}
#endif /* extern "C" */

#endif
//===========================================================================
// End of file.
//===========================================================================
