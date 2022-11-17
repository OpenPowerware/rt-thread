

#ifndef DSP2803x_HRCAP_H
#define DSP2803x_HRCAP_H

#ifdef __cplusplus
extern "C" {
#endif

//
//  High-Resolution Capture Register Bit Definitions
//
struct HCCTL_BITS {            // bit     description
    Uint32   SOFTRESET:1;      // 0       Soft Reset
    Uint32   RISEINTE:1;       // 1       RISE Capture Interrupt Enable Bit
    Uint32   FALLINTE:1;       // 2       FALL Capture Interrupt Enable Bit
    Uint32   OVFINTE:1;        // 3       Counter Overflow Interrupt Enable Bit
    Uint32   rsvd1:4;          // 7:4     reserved
    Uint32   HCCAPCLKSEL:1;    // 8       Capture Clock Select Bit
    Uint32   rsvd2:7; 		   // 15:9	  reserved
    Uint32   rsvd3:16;         // 31:16    reserved

};

union HCCTL_REG  {
    Uint32                       all;
    struct HCCTL_BITS            bit;
};

struct HCIFR_BITS {            // bit       description
    Uint32   INT:1;            // 0         Global Interrupt Flag
    Uint32   RISE:1;           // 1         RISE Capture Interrupt Flag
    Uint32   FALL:1;           // 2         FALL Capture Interrupt Flag
    Uint32   COUNTEROVF:1;     // 3         Counter Overflow Interrupt Flag
    Uint32   RISEOVF:1;  	   // 4         RISE Interrupt Overflow Event Flag
    Uint32   rsvd1:11;         // 15:5      reserved
    Uint32   rsvd2:16;         // 31:16    reserved
};

union HCIFR_REG  {
    Uint32                       all;
    struct HCIFR_BITS            bit;
};

struct HCICLR_BITS {           // bit description
    Uint32   INT:1;            // 0   Global Interrupt Flag Clear Bit
    Uint32   RISE:1;           // 1   RISE Capture Interrupt Flag Clear Bit
    Uint32   FALL:1;           // 2   FALL Capture Interrupt Flag Clear Bit
    Uint32   COUNTEROVF:1;     // 3   Counter Overflow Interrupt Flag Clear Bit
    Uint32   RISEOVF:1;  	   // 4   RISE Int. Overflow Event Flag Clear Bit
    Uint32   rsvd1:11;         // 15:5 reserved
    Uint32   rsvd2:16;         // 31:16    reserved
};

union HCICLR_REG  {
    Uint32                       all;
    struct HCICLR_BITS           bit;
};

struct HCIFRC_BITS {           // bit  description
    Uint32   rsvd1:1;          // 0    reserved
    Uint32   RISE:1;           // 1    RISE Capture Interrupt Flag Force Bit
    Uint32   FALL:1;           // 2    FALL Capture Interrupt Flag Force Bit
    Uint32   COUNTEROVF:1;     // 3    Counter Overflow Interrupt Flag Force Bit
    Uint32   RISEOVF:1;  	   // 4    RISE Int. Overflow Event Flag Force Bit
    Uint32   rsvd2:11;         // 15:5 reserved
    Uint32   rsvd3:16;         // 31:16    reserved
};

union HCIFRC_REG  {
    Uint32                       all;
    struct HCIFRC_BITS           bit;
};


struct HCCAL_BITS {                  // bit     description
    Uint32   DLL_START_POINT:8;      // 7:0     Default:0x20
    Uint32   PHASE_DETECT_SEL:3;     // 10:8    Default:0x0
    Uint32   rsvd:3;                 // 13:11   reserved
    Uint32   HRCAPMODE:1;            // 14      capture mode selection 
    Uint32   HRPWMSEL:1;             // 15      HRPWM selection    
    Uint32   rsvd2:16;               // 31:16   reserved
};

union HCCAL_REG  {
    Uint32                       all;
    struct HCCAL_BITS           bit;
};


struct HCMEPSTATUS_BITS {            // bit     description
    Uint32   LOCK_STATUS:2;          // 1:0     Value:0-unlock; 1-lock_done; 2/3-lock error
    Uint32   MEP_SCALE_FACTOR:8;     // 9:2     
    Uint32   rsvd:6;                 // 15:10   reserved     
    Uint32   rsvd2:16;               // 31:16   reserved
};

union HCMEPSTATUS_REG  {
    Uint32                       all;
    struct HCMEPSTATUS_BITS           bit;
};




struct HRCAP_REGS {
    union  HCCTL_REG           HCCTL;            // h0 0x00
    union  HCIFR_REG           HCIFR;            // h1 0x04
    union  HCICLR_REG          HCICLR;           // h2 0x08
    union  HCIFRC_REG          HCIFRC;           // h3 0x0C
    Uint32                     HCCOUNTER;        // h4 0x10
    union  HCCAL_REG           HCCAL;            // h5 0x14
    Uint32                     HCCALMEP;         // h6 0x18
    union  HCMEPSTATUS_REG     HCMEPSTATUS;      // h7 0x1C
    Uint32                     rsvd0[8];         
    Uint32                     HCCAPCNTRISE0;    // h10 0x40
    Uint32                     rsvd2[1];            
    Uint32                     HCCAPCNTFALL0;    // h12 0x48
    Uint32                     rsvd3[5];
    Uint32                     HCCAPCNTRISE1;    // h18 0x60
    Uint32                     rsvd4[1];
    Uint32                     HCCAPCNTFALL1;    // h1A 0x68
};

//
//  High Resolution Capture External References and Function Declarations
//
extern volatile struct HRCAP_REGS *const P_HRCap1Regs;
extern volatile struct HRCAP_REGS *const P_HRCap2Regs;
extern volatile struct HRCAP_REGS HRCap1Regs;
extern volatile struct HRCAP_REGS HRCap2Regs;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  

//
// End of file
//

