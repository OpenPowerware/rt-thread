

#ifndef DSP2803x_EQEP_H
#define DSP2803x_EQEP_H

#ifdef __cplusplus
extern "C" {
#endif

    
    
//---------------------------------------------------------------------------
// EQEP Individual Register Bit Definitions:

struct QDECCTL_BITS {            // bits description
        Uint32 rsvd1 : 5;        // 4:0   Reserved
        Uint32 QSP : 1;          // 5     QEPS Input Polarity
        Uint32 QIP : 1;          // 6     QEPI Input Polarity
        Uint32 QBP : 1;          // 7     QEPB Input Polarity
        Uint32 QAP : 1;          // 8     QEPA Input Polarity
        Uint32 IGATE : 1;        // 9     Index Pulse Gating Option
        Uint32 SWAP : 1;         // 10    CLK/DIR Signal Source for Position
                                 //       Counter
        Uint32 XCR : 1;          // 11    External Clock Rate
        Uint32 SPSEL : 1;        // 12    Sync Output Pin Select
        Uint32 SOEN : 1;         // 13    Enable Position Compare Sync
        Uint32 QSRC : 2;         // 15:14 Postion Counter Source
        	
        Uint32 rsvd2 : 16;        // 31:16   Reserved
};

union QDECCTL_REG {
        Uint32 all;
        struct QDECCTL_BITS bit;
};

struct QEPCTL_BITS {             // bits description
        Uint32 WDE : 1;          // 0     QEP watchdog enable
        Uint32 UTE : 1;          // 1     QEP unit timer enable
        Uint32 QCLM : 1;         // 2     QEP capture latch mode
        Uint32 QPEN : 1;         // 3     Quadrature postotion counter enable
        Uint32 IEL : 2;          // 5:4   Index event latch
        Uint32 SEL : 1;          // 6     Strobe event latch
        Uint32 SWI : 1;          // 7     Software init position counter
        Uint32 IEI : 2;          // 9:8   Index event init of position count
        Uint32 SEI : 2;          // 11:10 Strobe event init
        Uint32 PCRM : 2;         // 13:12 Postion counter reset
        Uint32 FREE_SOFT : 2;    // 15:14 Emulation mode
        	
        Uint32 rsvd1 : 16;        // 31:16   Reserved
};

union QEPCTL_REG {
        Uint32 all;
        struct QEPCTL_BITS bit;
};

struct QCAPCTL_BITS {            // bits description
        Uint32 UPPS : 4;         // 3:0  Unit position pre-scale
        Uint32 CCPS : 3;         // 6:4  QEP capture timer pre-scale
        Uint32 rsvd1 : 8;        // 14:7 Reserved
        Uint32 CEN : 1;          // 15   Enable QEP capture
        
        Uint32 rsvd2 : 16;        // 31:16   Reserved
};

union QCAPCTL_REG {
        Uint32 all;
        struct QCAPCTL_BITS bit;
};

struct QPOSCTL_BITS {           // bits description
        Uint32 PCSPW : 12;      // 11:0  Position compare sync pulse width
        Uint32 PCE : 1;         // 12    Position compare enable/disable
        Uint32 PCPOL : 1;       // 13    Polarity of sync output
        Uint32 PCLOAD : 1;      // 14    Position compare of shadow load
        Uint32 PCSHDW : 1;      // 15    Position compare of shadow enable
        	
        Uint32 rsvd1 : 16;        // 31:16   Reserved
};

union QPOSCTL_REG {
        Uint32 all;
        struct QPOSCTL_BITS bit;
};

struct QEINT_BITS {             // bits description
        Uint32 rsvd1 : 1;       // 0     Reserved
        Uint32 PCE : 1;         // 1     Position counter error
        Uint32 QPE : 1;         // 2     Quadrature phase error     //phe in verilog
        Uint32 QDC : 1;         // 3     Quadrature dir change
        Uint32 WTO : 1;         // 4     Watchdog timeout
        Uint32 PCU : 1;         // 5     Position counter underflow
        Uint32 PCO : 1;         // 6     Position counter overflow
        Uint32 PCR : 1;         // 7     Position compare ready
        Uint32 PCM : 1;         // 8     Position compare match
        Uint32 SEL : 1;         // 9     Strobe event latch
        Uint32 IEL : 1;         // 10    Event latch
        Uint32 UTO : 1;         // 11    Unit timeout
        Uint32 rsvd2 : 20;       // 31:12 Reserved
};

union QEINT_REG {
        Uint32 all;
        struct QEINT_BITS bit;
};

struct QFLG_BITS {              // bits description
        Uint32 INT : 1;         // 0     Global.interrupt
        Uint32 PCE : 1;         // 1     Position counter error
        Uint32 PHE : 1;         // 2     Quadrature phase error
        Uint32 QDC : 1;         // 3     Quadrature dir change
        Uint32 WTO : 1;         // 4     Watchdog timeout
        Uint32 PCU : 1;         // 5     Position counter underflow
        Uint32 PCO : 1;         // 6     Position counter overflow
        Uint32 PCR : 1;         // 7     Position compare ready
        Uint32 PCM : 1;         // 8     Position compare match
        Uint32 SEL : 1;         // 9     Strobe event latch
        Uint32 IEL : 1;         // 10    Event latch
        Uint32 UTO : 1;         // 11    Unit timeout
        Uint32 rsvd1 : 20;       // 31:12 Reserved
};

union QFLG_REG {
        Uint32 all;
        struct QFLG_BITS bit;
};

struct QFRC_BITS {              // bits description
        Uint32 rsvd1 : 1;       // 0     Reserved
        Uint32 PCE : 1;         // 1     Position counter error
        Uint32 PHE : 1;         // 2     Quadrature phase error
        Uint32 QDC : 1;         // 3     Quadrature dir change
        Uint32 WTO : 1;         // 4     Watchdog timeout
        Uint32 PCU : 1;         // 5     Position counter underflow
        Uint32 PCO : 1;         // 6     Position counter overflow
        Uint32 PCR : 1;         // 7     Position compare ready
        Uint32 PCM : 1;         // 8     Position compare match
        Uint32 SEL : 1;         // 9     Strobe event latch
        Uint32 IEL : 1;         // 10    Event latch
        Uint32 UTO : 1;         // 11    Unit timeout
        Uint32 rsvd2 : 20;       // 31:12 Reserved
};

union QFRC_REG {
        Uint32 all;
        struct QFRC_BITS bit;
};

struct QEPSTS_BITS {             // bits description
        Uint32 PCEF : 1;         // 0    Position counter error
        Uint32 FIMF : 1;         // 1    First index marker
        Uint32 CDEF : 1;         // 2    Capture direction error
        Uint32 COEF : 1;         // 3    Capture overflow error
        Uint32 QDLF : 1;         // 4    QEP direction latch
        Uint32 QDF : 1;          // 5    Quadrature direction
        Uint32 FIDF : 1;         // 6    Direction on first index marker
        Uint32 UPEVNT : 1;       // 7    Unit position event flag
        Uint32 rsvd1 : 24;        // 31:8 Reserved
};

union QEPSTS_REG {
        Uint32 all;
        struct QEPSTS_BITS bit;
};

struct EQEP_REGS {
        Uint32 QPOSCNT;                   // Position Counter
        Uint32 QPOSINIT;                  // Position Counter Init
        Uint32 QPOSMAX;                   // Maximum Position Count
        Uint32 QPOSCMP;                   // Position Compare
        Uint32 QPOSILAT;                  // Index Position Latch
        Uint32 QPOSSLAT;                  // Strobe Position Latch
        Uint32 QPOSLAT;                   // Position Latch
        Uint32 QUTMR;                     // QEP Unit Timer
        Uint32 QUPRD;                     // QEP Unit Period
        Uint32 QWDTMR;                    // QEP Watchdog Timer
        Uint32 QWDPRD;                    // QEP Watchdog Period
        union    QDECCTL_REG QDECCTL;     // Quadrature Decoder Control
        union    QEPCTL_REG QEPCTL;       // QEP Control
        union    QCAPCTL_REG QCAPCTL;     // Qaudrature Capture Control
        union    QPOSCTL_REG QPOSCTL;     // Position Compare Control
        union    QEINT_REG QEINT;         // QEP Interrupt Control
        union    QFLG_REG QFLG;           // QEP Interrupt Flag
        union    QFLG_REG QCLR;           // QEP Interrupt Clear  //only get 0  in  verilog  ??
        union    QFRC_REG QFRC;           // QEP Interrupt Force  //only get 0  in  verilog  ??
        union    QEPSTS_REG QEPSTS;       // QEP Status
        Uint32 QCTMR;                     // QEP Capture Timer
        Uint32 QCPRD;                     // QEP Capture Period
        Uint32 QCTMRLAT;                  // QEP Capture Latch
        Uint32 QCPRDLAT;                  // QEP Capture Period Latch
        Uint32 rsvd1;                     // Reserved
};

//---------------------------------------------------------------------------
// Eqep External References & Function Declarations:
//extern volatile struct EQEP_REGS EQep1Regs;
//extern volatile struct EQEP_REGS EQep2Regs;
//extern volatile struct EQEP_REGS EQep3Regs;

extern volatile struct EQEP_REGS *const P_EQep1Regs;
extern volatile struct EQEP_REGS EQep1Regs;

extern volatile struct EQEP_REGS *const P_EQep2Regs;
extern volatile struct EQEP_REGS EQep2Regs;



#ifdef __cplusplus
}
#endif  /* extern "C" */


#endif  // end of F28M36x_EQEP_H definition


