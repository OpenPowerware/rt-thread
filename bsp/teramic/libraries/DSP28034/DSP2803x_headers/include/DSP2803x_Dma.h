#ifndef DSP2803x_DMA_H
#define DSP2803x_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DSC28034

struct DMACFGREG_BITS{
	Uint64 DMA_EN:1;  // 0 DW_ahb_dmac Enable bit.
	                 // 0 = DW_ahb_dmac Disabled
                     // 1 = DW_ahb_dmac Enabled.
	Uint64 rsvd:63;   // 63:1 Reserved
};
union DMACFGREG_REG{
	Uint64  all;
	struct DMACFGREG_BITS bit;
};

//====================================================

struct CHENREG_BITS{
	Uint64 CH0_EN:1;  // 0 Enables/Disables the channel. 
	Uint64 CH1_EN:1;  // 1 Enables/Disables the channel.
	Uint64 CH2_EN:1;  // 2 Enables/Disables the channel.
	Uint64 CH3_EN:1;  // 3 Enables/Disables the channel. 
	Uint64 rsvd:4;   // 7:4 Reserved
	Uint64 CH0_EN_WE:1;   // 8  Channel enable write enable.
	Uint64 CH1_EN_WE:1;   // 9  Channel enable write enable.
	Uint64 CH2_EN_WE:1;   // 10 Channel enable write enable.
	Uint64 CH3_EN_WE:1;   // 11 Channel enable write enable.
	Uint64 rsvd1:52;   // 63:12 Reserved
};
union CHENREG_REG{
	Uint64  all;
	struct CHENREG_BITS bit;
};

//====================================================

struct SAR_BITS{
	Uint64  SAR:32;   // 31:0 Current Source Address of DMA transfer
	Uint64  rsvd:32;  // 63:32 Reserved
};
union SAR_REG{
	Uint64  all;
	struct SAR_BITS bit;
};

//====================================================

struct DAR_BITS{
	Uint64  DAR:32;   // 31:0 Current Destination address of DMA transfer
	Uint64  rsvd:32;  // 63:32 Reserved
};
union DAR_REG{
	Uint64  all;
	struct DAR_BITS bit;
};

//====================================================

struct CTL_BITS{
    Uint64  INT_EN:1;          //0 Interrupt Enable Bit
    Uint64  DST_TR_WIDTH:3;    // 3:1 Destination Transfer Width
    Uint64  SRC_TR_WIDTH:3;    // 6:4 Source Transfer Width
    Uint64  DINC:2;            // 8:7 Destination Address Increment
    Uint64  SINC:2;            // 10:9 Source Address Increment
    Uint64  DEST_MSIZE:3;      // 13:11 Destination Burst Transaction Length
    Uint64  SRC_MSIZE:3;       // 16:14 Source Burst Transaction Length
    Uint64  rsvd:3;            // 19:17 Reserved
    Uint64  TT_FC:3;           //22:20 Transfer Type and Flow Control(bit22==0)
    Uint64  rsvd1:9;           // 31:23 Reserved
    Uint64  BLOCK_TS:12;       // 43:32 Block Transfer Size
    Uint64  DONE:1;            // 44 Done bit
    Uint64  rsvd2:19;          // 63:45 Reserved
};
union CTL_REG{
	Uint64  all;
	struct CTL_BITS bit;
};

//====================================================

struct CFG_BITS{
    Uint64  rsvd:5;           // 4:0 Reserved
    Uint64  CH_PRIOR:3;       // 7:5 Channel priority
    Uint64  CH_SUSP:1;        // 8 Channel Suspend
    Uint64  FIFO_EMPTY:1;     // 9  Indicates if there is data left in the channel FIFO
    Uint64  HS_SEL_DST:1;     // 10 Destination Software or Hardware Handshaking Select
    Uint64  HS_SEL_SRC:1;     // 11 Source Software or Hardware Handshaking Select
    Uint64  rsvd1:2;          // 13:12 Reserved
    Uint64  rsvd2:2;          // 15:14 Reserved
    Uint64  rsvd3:1;          // 16 Reserved
    Uint64  rsvd4:1;          // 17 Reserved
    Uint64  DST_HS_POL:1;     // 18 Destination Handshaking Interface Polarity
    Uint64  SRC_HS_POL:1;     // 19 Source Handshaking Interface Polarity
    Uint64  rsvd5:10;         // 29:20 Reserved
    Uint64  rsvd6:1;          // 30 Reserved  
    Uint64  rsvd7:1;          // 31 Reserved  
    Uint64  FCMODE:1;         // 32 Flow Control Mode
    Uint64  FIFO_MODE:1;      // 33 FIFO Mode Select
    Uint64  PROTCTL:3;        // 36:34 Protection Control bits
    Uint64  rsvd8:1;          // 37 Reserved
    Uint64  rsvd9:1;          // 38 Reserved
    Uint64  SRC_PER:2;        // 40:39 Assigns a hardware handshaking interface to the destination of channel x(b=2)
    Uint64  rsvd10:2;         // 42:41 Reserved
    Uint64  DEST_PER:2;       // 44:43 Assigns a hardware handshaking interface to the source of channel x
    Uint64  rsvd11:19;        // 63:45 Reserved
};
union CFG_REG{
	Uint64  all;
	struct CFG_BITS bit;
};

//====================================================

struct CH_REGS {
    union SAR_REG 		SAR;		//Source Address Register for Channel x  
	union DAR_REG 		DAR;		//Destination Address Register for Channel x
	Uint64  rsvd;                   // Reserved  
	union CTL_REG		CTL;		// Control Register for Channel x
	Uint64  rsvd1;                  // Reserved
	Uint64  rsvd2;                  // Reserved     
	Uint64  rsvd3;                  // Reserved
	Uint64  rsvd4;                  // Reserved
	union CFG_REG		CFG;		//40  RW
	Uint64  rsvd5;                  // Reserved
	Uint64  rsvd6;                  // Reserved
};

////////////////////////////////////////////////////////
//====================================================

struct RAW_BITS{
	Uint64  RAW:4;     // 3:0 Raw interrupt status
	Uint64  rsvd:60;   // 63:4 Reserved
};
union RAW_REGS{
	Uint64  all;
	struct RAW_BITS bit;
};

//====================================================

struct STATUS_BITS{
	Uint64  STATUS:4;   // 3:0 Current Destination address of DMA transfer
	Uint64  rsvd:60;    // 63:4 Reserved
};
union STATUS_REGS{
	Uint64  all;
	struct STATUS_BITS bit;
};

//====================================================

struct MASK_BITS{
	Uint64  INT_MASK:4;     // 3:0 Interrupt Mask
	Uint64  rsvd:4;         // 7:4 Reserved
	Uint64  INT_MASK_WE:4;  //11:8 Interrupt Mask Write Enable
	Uint64  rsvd1:52;       // 63:12 Reserved
};
union MASK_REGS{
	Uint64  all;
	struct MASK_BITS bit;
};

//====================================================

struct CLEAR_BITS{
	Uint64  CLEAR:4;   // 3:0 Interrupt clear
	Uint64  rsvd:60;   // 63:4 Reserved
};
union CLEAR_REGS{
	Uint64  all;
	struct CLEAR_BITS bit;
};

//////////////////////////////////////////////////////
//====================================================

struct STATUSINT_BITS{
	Uint64  TFR:1;     // 0 OR of the contents of StatusTfr register
	Uint64  BLOCK:1;   // 1 OR of the contents of StatusBlock register
	Uint64  SRCT:1;    // 2 OR of the contents of StatusSrcTran register
	Uint64  DSTT:1;    // 3  OR of the contents of StatusDst register
	Uint64  ERR:1;     // 4  OR of the contents of StatusErr register
	Uint64  rsvd:59;   // 63:5 Reserved
};
union STATUSINT_REG{
	Uint64  all;
	struct STATUSINT_BITS bit;
};

//////////////////////////////////////////////////////
//====================================================

struct REQSRCREG_BITS{
	Uint64  SRC_REQ:4;     // 3:0 Source request
	Uint64  rsvd:4;        // 7:4 Reserved
	Uint64  SRC_REQ_WE:4;  // 11:8  Source request write enable
	Uint64  rsvd1:52;      // 63:12 Reserved
};
union REQSRCREG_REG{
	Uint64  all;
	struct REQSRCREG_BITS bit;
};

//====================================================

struct REQDSTREG_BITS{
	Uint64  DST_REQ:4;     // 3:0 Destination request
	Uint64  rsvd:4;        // 7:4 Reserved
	Uint64  DST_REQ_WE:4;  // 11:8  Destination request write enable
	Uint64  rsvd1:52;      // 63:12 Reserved
};
union REQDSTREG_REG{
	Uint64  all;
	struct REQDSTREG_BITS bit;
};

//====================================================

struct SGLREQSRCREG_BITS{
	Uint64  SRC_SGLREQ:4;   // 3:0 Source single request
	Uint64  rsvd:4;         // 7:4 Reserved
	Uint64  SRC_REQ_WE:4;   // 11:8  Single write enable
	Uint64  rsvd1:52;       // 63:12 Reserved
};
union SGLREQSRCREG_REG{
	Uint64  all;
	struct SGLREQSRCREG_BITS bit;
};

//====================================================

struct SGLREQDSTREG_BITS{
	Uint64  DST_SGLREQ:4;     // 3:0  Destination single or burst request
	Uint64  rsvd:4;           // 7:4 Reserved
	Uint64  DST_SGLREQ_WE:4;  // 11:8  Destination write enable
	Uint64  rsvd1:52;         // 63:12 Reserved
};
union SGLREQDSTREG_REG{
	Uint64  all;
	struct SGLREQDSTREG_BITS bit;
};

//====================================================

struct LSTSRCREG_BITS{
	Uint64  LSTSRC:4;     // 3:0   Source last transaction request
	Uint64  rsvd:4;       // 7:4 Reserved
	Uint64  LSTSRC_WE:4;  // 11:8  Source last transaction request write enable
	Uint64  rsvd1:52;     // 63:12 Reserved
};
union LSTSRCREG_REG{
	Uint64  all;
	struct SGLREQDSTREG_BITS bit;
};

//====================================================

struct LSTDSTREG_BITS{
	Uint64  LSTDST:4;     // 3:0   Destination last transaction request
	Uint64  rsvd:4;       // 7:4 Reserved
	Uint64  LSTDST_WE:4;  // 11:8   Destination last transaction request write enable
	Uint64  rsvd1:52;     // 63:12 Reserved
};
union LSTDSTREG_REG{
	Uint64  all;
	struct LSTDSTREG_BITS bit;
};

//////////////////////////////////////////////////////
//====================================================

struct DMAIDREG_BITS{
	Uint64  DMAH_ID:32;   // 31:0   Hardcoded DW_ahb_dmac Peripheral ID
	Uint64  rsvd:32;      // 63:32 Reserved
};
union DMAIDREG_REG{
	Uint64  all;
	struct DMAIDREG_BITS bit;
};

//====================================================

struct DMATESTREG_BITS{
	Uint64  TEST_SLV_IF:1;   // 0   Puts the AHB slave interface into test mode
	Uint64  rsvd:63;         // 63:1 Reserved
};
union DMATESTREG_REG{
	Uint64  all;
	struct DMATESTREG_BITS bit;
};

//////////////////////////////////////////////////////
//====================================================

struct INTCTL_REGS{
	Uint32  CH_INT_CTL:6;   // 5:0 
	Uint32  rsvd:2;         // 7:6    Reserved
	Uint32  CH_INT_EN:1;    // 8 
	Uint32  rsvd2:7;        // 15:9   Reserved
	Uint32  R_DMA_REQ:1;    // 16
	Uint32  rsvd3:7;        // 23:17  Reserved
	Uint32  CH_INT_CLR:1;   // 24 
	Uint32  rsvd4:7;        // 31:25   Reserved      	
};

///////////////////////////////////////////////////////
//====================================================

struct DMA_COMP_PARAMS_6_BITS{
	
	Uint64  rsvd:32;                //31:0 Reserved
	Uint64  CH7_DTW:3;              //34:32 DMAH_CHx_STW
	Uint64  CH7_STW:3;              //37:35 DMAH_CHx_DTW
	Uint64  CH7_STAT_DST:1;         //38 DMAH_CHx_STAT_DST
	Uint64  CH7_STAT_SRC:1;         //39 DMAH_CHX_STAT_SRC
	Uint64  CH7_DST_SCA_EN:1;       //40 DMAH_CHx_DST_SCA_EN
	Uint64  CH7_SRC_GAT_EN:1;       //41 DMAH_CHx_SRC_GAT_EN
	Uint64  CH7_LOCK_EN:1;          //42 DMAH_CHx_LOCK_EN
	Uint64  CH7_MULTI_BLK_EN:1;     //43 DMAH_CHx_MULTI_BLK_EN
	Uint64  CH7_CTL_WB_EN:1;        //44 DMAH_CHx_CTL_WB_EN
	Uint64  CH7_HC_LLP:1;           //45 DMAH_CHx_HC_LLP
	Uint64  CH7_FC:2;               //47:46 DMAH_CHx_FC
	Uint64  CH7_MAX_MULT_SIZE:3;    //50:48 DMAH_CHx_MULT_SIZE
	Uint64  CH7_DMS:3;              //53:51 DMAH_CHx_DMS
	Uint64  CH7_LMS:3;              //56:54 DMAH_CHx_LMS
	Uint64  CH7_SMS:3;              //59:57 DMAH_CHx_SMS
	Uint64  CH7_FIFO_DEPTH:3;       //62:60 DMAH_CHx_FIFO_DEPTH
	Uint64  rsvd1:1;                //63;    
};
union DMA_COMP_PARAMS_6_REG{
	Uint64  all;
	struct DMA_COMP_PARAMS_6_BITS bit;
};

//====================================================

struct DMA_COMP_PARAMS_5_BITS{
	

	Uint64  CH6_DTW:3;              //2:0 DMAH_CHx_STW
	Uint64  CH6_STW:3;              //5:3 DMAH_CHx_DTW
	Uint64  CH6_STAT_DST:1;         //6 DMAH_CHx_STAT_DST
	Uint64  CH6_STAT_SRC:1;         //7 DMAH_CHX_STAT_SRC
	Uint64  CH6_DST_SCA_EN:1;       //8 DMAH_CHx_DST_SCA_EN
	Uint64  CH6_SRC_GAT_EN:1;       //9 DMAH_CHx_SRC_GAT_EN
	Uint64  CH6_LOCK_EN:1;          //10 DMAH_CHx_LOCK_EN
	Uint64  CH6_MULTI_BLK_EN:1;     //11 DMAH_CHx_MULTI_BLK_EN
	Uint64  CH6_CTL_WB_EN:1;        //12 DMAH_CHx_CTL_WB_EN
	Uint64  CH6_HC_LLP:1;           //13 DMAH_CHx_HC_LLP
	Uint64  CH6_FC:2;               //15:14 DMAH_CHx_FC
	Uint64  CH6_MAX_MULT_SIZE:3;    //18:16 DMAH_CHx_MULT_SIZE
	Uint64  CH6_DMS:3;              //21:19 DMAH_CHx_DMS
	Uint64  CH6_LMS:3;              //24:22 DMAH_CHx_LMS
	Uint64  CH6_SMS:3;              //27:25 DMAH_CHx_SMS
	Uint64  CH6_FIFO_DEPTH:3;       //30:28 DMAH_CHx_FIFO_DEPTH
	Uint64  rsvd:1;                 //31;

	Uint64  CH5_DTW:3;              //34:32 DMAH_CHx_STW
	Uint64  CH5_STW:3;              //37:35 DMAH_CHx_DTW
	Uint64  CH5_STAT_DST:1;         //38 DMAH_CHx_STAT_DST
	Uint64  CH5_STAT_SRC:1;         //39 DMAH_CHX_STAT_SRC
	Uint64  CH5_DST_SCA_EN:1;       //40 DMAH_CHx_DST_SCA_EN
	Uint64  CH5_SRC_GAT_EN:1;       //41 DMAH_CHx_SRC_GAT_EN
	Uint64  CH5_LOCK_EN:1;          //42 DMAH_CHx_LOCK_EN
	Uint64  CH5_MULTI_BLK_EN:1;     //43 DMAH_CHx_MULTI_BLK_EN
	Uint64  CH5_CTL_WB_EN:1;        //44 DMAH_CHx_CTL_WB_EN
	Uint64  CH5_HC_LLP:1;           //45 DMAH_CHx_HC_LLP
	Uint64  CH5_FC:2;               //47:46 DMAH_CHx_FC
	Uint64  CH5_MAX_MULT_SIZE:3;    //50:48 DMAH_CHx_MULT_SIZE
	Uint64  CH5_DMS:3;              //53:51 DMAH_CHx_DMS
	Uint64  CH5_LMS:3;              //56:54 DMAH_CHx_LMS
	Uint64  CH5_SMS:3;              //59:57 DMAH_CHx_SMS
	Uint64  CH5_FIFO_DEPTH:3;       //62:60 DMAH_CHx_FIFO_DEPTH
	Uint64  rsvd1:1;                //63; 

};
union DMA_COMP_PARAMS_5_REG{
	Uint64  all;
	struct DMA_COMP_PARAMS_5_BITS bit;
};

//====================================================

struct DMA_COMP_PARAMS_4_BITS{
	
	Uint64  CH4_DTW:3;              //2:0 DMAH_CHx_STW
	Uint64  CH4_STW:3;              //5:3 DMAH_CHx_DTW
	Uint64  CH4_STAT_DST:1;         //6 DMAH_CHx_STAT_DST
	Uint64  CH4_STAT_SRC:1;         //7 DMAH_CHX_STAT_SRC
	Uint64  CH4_DST_SCA_EN:1;       //8 DMAH_CHx_DST_SCA_EN
	Uint64  CH4_SRC_GAT_EN:1;       //9 DMAH_CHx_SRC_GAT_EN
	Uint64  CH4_LOCK_EN:1;          //10 DMAH_CHx_LOCK_EN
	Uint64  CH4_MULTI_BLK_EN:1;     //11 DMAH_CHx_MULTI_BLK_EN
	Uint64  CH4_CTL_WB_EN:1;        //12 DMAH_CHx_CTL_WB_EN
	Uint64  CH4_HC_LLP:1;           //13 DMAH_CHx_HC_LLP
	Uint64  CH4_FC:2;               //15:14 DMAH_CHx_FC
	Uint64  CH4_MAX_MULT_SIZE:3;    //18:16 DMAH_CHx_MULT_SIZE
	Uint64  CH4_DMS:3;              //21:19 DMAH_CHx_DMS
	Uint64  CH4_LMS:3;              //24:22 DMAH_CHx_LMS
	Uint64  CH4_SMS:3;              //27:25 DMAH_CHx_SMS
	Uint64  CH4_FIFO_DEPTH:3;       //30:28 DMAH_CHx_FIFO_DEPTH
	Uint64  rsvd:1;                 //31;

	Uint64  CH3_DTW:3;              //34:32 DMAH_CHx_STW
	Uint64  CH3_STW:3;              //37:35 DMAH_CHx_DTW
	Uint64  CH3_STAT_DST:1;         //38 DMAH_CHx_STAT_DST
	Uint64  CH3_STAT_SRC:1;         //39 DMAH_CHX_STAT_SRC
	Uint64  CH3_DST_SCA_EN:1;       //40 DMAH_CHx_DST_SCA_EN
	Uint64  CH3_SRC_GAT_EN:1;       //41 DMAH_CHx_SRC_GAT_EN
	Uint64  CH3_LOCK_EN:1;          //42 DMAH_CHx_LOCK_EN
	Uint64  CH3_MULTI_BLK_EN:1;     //43 DMAH_CHx_MULTI_BLK_EN
	Uint64  CH3_CTL_WB_EN:1;        //44 DMAH_CHx_CTL_WB_EN
	Uint64  CH3_HC_LLP:1;           //45 DMAH_CHx_HC_LLP
	Uint64  CH3_FC:2;               //47:46 DMAH_CHx_FC
	Uint64  CH3_MAX_MULT_SIZE:3;    //50:48 DMAH_CHx_MULT_SIZE
	Uint64  CH3_DMS:3;              //53:51 DMAH_CHx_DMS
	Uint64  CH3_LMS:3;              //56:54 DMAH_CHx_LMS
	Uint64  CH3_SMS:3;              //59:57 DMAH_CHx_SMS
	Uint64  CH3_FIFO_DEPTH:3;       //62:60 DMAH_CHx_FIFO_DEPTH
	Uint64  rsvd1:1;                //63;

};
union DMA_COMP_PARAMS_4_REG{
	Uint64  all;
	struct DMA_COMP_PARAMS_4_BITS bit;
};

//====================================================

struct DMA_COMP_PARAMS_3_BITS{
	
	Uint64  CH2_DTW:3;              //2:0 DMAH_CHx_STW
	Uint64  CH2_STW:3;              //5:3 DMAH_CHx_DTW
	Uint64  CH2_STAT_DST:1;         //6 DMAH_CHx_STAT_DST
	Uint64  CH2_STAT_SRC:1;         //7 DMAH_CHX_STAT_SRC
	Uint64  CH2_DST_SCA_EN:1;       //8 DMAH_CHx_DST_SCA_EN
	Uint64  CH2_SRC_GAT_EN:1;       //9 DMAH_CHx_SRC_GAT_EN
	Uint64  CH2_LOCK_EN:1;          //10 DMAH_CHx_LOCK_EN
	Uint64  CH2_MULTI_BLK_EN:1;     //11 DMAH_CHx_MULTI_BLK_EN
	Uint64  CH2_CTL_WB_EN:1;        //12 DMAH_CHx_CTL_WB_EN
	Uint64  CH2_HC_LLP:1;           //13 DMAH_CHx_HC_LLP
	Uint64  CH2_FC:2;               //15:14 DMAH_CHx_FC
	Uint64  CH2_MAX_MULT_SIZE:3;    //18:16 DMAH_CHx_MULT_SIZE
	Uint64  CH2_DMS:3;              //21:19 DMAH_CHx_DMS
	Uint64  CH2_LMS:3;              //24:22 DMAH_CHx_LMS
	Uint64  CH2_SMS:3;              //27:25 DMAH_CHx_SMS
	Uint64  CH2_FIFO_DEPTH:3;       //30:28 DMAH_CHx_FIFO_DEPTH
	Uint64  rsvd:1;                 //31;

	Uint64  CH1_DTW:3;              //34:32 DMAH_CHx_STW
	Uint64  CH1_STW:3;              //37:35 DMAH_CHx_DTW
	Uint64  CH1_STAT_DST:1;         //38 DMAH_CHx_STAT_DST
	Uint64  CH1_STAT_SRC:1;         //39 DMAH_CHX_STAT_SRC
	Uint64  CH1_DST_SCA_EN:1;       //40 DMAH_CHx_DST_SCA_EN
	Uint64  CH1_SRC_GAT_EN:1;       //41 DMAH_CHx_SRC_GAT_EN
	Uint64  CH1_LOCK_EN:1;          //42 DMAH_CHx_LOCK_EN
	Uint64  CH1_MULTI_BLK_EN:1;     //43 DMAH_CHx_MULTI_BLK_EN
	Uint64  CH1_CTL_WB_EN:1;        //44 DMAH_CHx_CTL_WB_EN
	Uint64  CH1_HC_LLP:1;           //45 DMAH_CHx_HC_LLP
	Uint64  CH1_FC:2;               //47:46 DMAH_CHx_FC
	Uint64  CH1_MAX_MULT_SIZE:3;    //50:48 DMAH_CHx_MULT_SIZE
	Uint64  CH1_DMS:3;              //53:51 DMAH_CHx_DMS
	Uint64  CH1_LMS:3;              //56:54 DMAH_CHx_LMS
	Uint64  CH1_SMS:3;              //59:57 DMAH_CHx_SMS
	Uint64  CH1_FIFO_DEPTH:3;       //62:60 DMAH_CHx_FIFO_DEPTH
	Uint64  rsvd1:1;                //63
};
union DMA_COMP_PARAMS_3_REG{
	Uint64  all;
	struct DMA_COMP_PARAMS_3_BITS bit;
};

//====================================================

struct DMA_COMP_PARAMS_2_BITS{
	
	Uint64  CH0_DTW:3;             //2:0 DMAH_CHx_STW
	Uint64  CH0_STW:3;             //5:3 DMAH_CHx_DTW
	Uint64  CH0_STAT_DST:1;        //6 DMAH_CHx_STAT_DST
	Uint64  CH0_STAT_SRC:1;        //7 DMAH_CHX_STAT_SRC
	Uint64  CH0_DST_SCA_EN:1;      //8 DMAH_CHx_DST_SCA_EN
	Uint64  CH0_SRC_GAT_EN:1;      //9 DMAH_CHx_SRC_GAT_EN
	Uint64  CH0_LOCK_EN:1;         //10 DMAH_CHx_LOCK_EN
	Uint64  CH0_MULTI_BLK_EN:1;    //11 DMAH_CHx_MULTI_BLK_EN
	Uint64  CH0_CTL_WB_EN:1;       //12 DMAH_CHx_CTL_WB_EN
	Uint64  CH0_HC_LLP:1;          //13 DMAH_CHx_HC_LLP
	Uint64  CH0_FC:2;              //15:14 DMAH_CHx_FC
	Uint64  CH0_MAX_MULT_SIZE:3;   //18:16 DMAH_CHx_MULT_SIZE
	Uint64  CH0_DMS:3;             //21:19 DMAH_CHx_DMS
	Uint64  CH0_LMS:3;             //24:22 DMAH_CHx_LMS
	Uint64  CH0_SMS:3;             //27:25 DMAH_CHx_SMS
	Uint64  CH0_FIFO_DEPTH:3;      //30:28 DMAH_CHx_FIFO_DEPTH
	Uint64  rsvd:1;    //31;

	Uint64  CH0_MULTI_BLK_TYPE:4;  //35:32 DMAH_CHx_MULTI_BLK_TYPE
	Uint64  CH1_MULTI_BLK_TYPE:4;  //39:36 DMAH_CHx_MULTI_BLK_TYPE
	Uint64  CH2_MULTI_BLK_TYPE:4;  //43:40 DMAH_CHx_MULTI_BLK_TYPE
	Uint64  CH3_MULTI_BLK_TYPE:4;  //47:44 DMAH_CHx_MULTI_BLK_TYPE
	Uint64  CH4_MULTI_BLK_TYPE:4;  //51:48 DMAH_CHx_MULTI_BLK_TYPE
	Uint64  CH5_MULTI_BLK_TYPE:4;  //55:52 DMAH_CHx_MULTI_BLK_TYPE
	Uint64  CH6_MULTI_BLK_TYPE:4;  //59:56 DMAH_CHx_MULTI_BLK_TYPE
	Uint64  CH7_MULTI_BLK_TYPE:4;  //63:60 DMAH_CHx_MULTI_BLK_TYPE

};
union DMA_COMP_PARAMS_2_REG{
	Uint64  all;
	struct DMA_COMP_PARAMS_2_BITS bit;
};

//====================================================

struct DMA_COMP_PARAMS_1_BITS{
	
	Uint64  CH0_MAX_BLK_SIZE:4;        // 3:0 DMAH_CHx_MAX_BLK_SIZE
	Uint64  CH1_MAX_BLK_SIZE:4;        // 7:4 DMAH_CHx_MAX_BLK_SIZE
	Uint64  CH2_MAX_BLK_SIZE:4;        // 11:8 DMAH_CHx_MAX_BLK_SIZE
	Uint64  CH3_MAX_BLK_SIZE:4;        // 15:12 DMAH_CHx_MAX_BLK_SIZE
	Uint64  CH4_MAX_BLK_SIZE:4;        // 19:16 DMAH_CHx_MAX_BLK_SIZE
	Uint64  CH5_MAX_BLK_SIZE:4;        // 23:20 DMAH_CHx_MAX_BLK_SIZE
	Uint64  CH6_MAX_BLK_SIZE:4;        // 27:24 DMAH_CHx_MAX_BLK_SIZE
	Uint64  CH7_MAX_BLK_SIZE:4;        // 31:28 DMAH_CHx_MAX_BLK_SIZE

	Uint64  DMAH_BIG_ENDIAN:1;         // 32 DMAH_BIG_ENDIAN
	Uint64  INTR_IO:2;                 // 34:33 DMAH_INTR_IO
	Uint64  MABRST:1;                  // 35 DMAH_MABRST
	Uint64  rsvd:4;                    // 39:36 Reserved
	Uint64  NUM_CHANNELS:3;            // 42:40 DMAH_NUM_CHANNELS
	Uint64  NUM_MASTER_INT:2;          // 44:43 DMAH_NUM_MASTER_INT
	Uint64  S_HDATA_WIDTH:2;           // 46:45 DMAH_S_HDATA_WIDTH
	Uint64  M4_HDATA_WIDTH:2;          // 48:47 DMAH_M4_HDATA_WIDTH
	Uint64  M3_HDATA_WIDTH:2;          // 50:49 DMAH_M3_HDATA_WIDTH
	Uint64  M2_HDATA_WIDTH:2;          // 52:51 DMAH_M2_HDATA_WIDTH
	Uint64  M1_HDATA_WIDTH:2;          // 54:53 DMAH_M1_HDATA_WIDTH
	Uint64  NUM_HS_INT:5;              // 59:55 DMAH_NUM_HS_INT
	Uint64  ADD_ENCODED_PARAMS:1;      // 60 DMAH_ADD_ENCODED_PARAMS
	Uint64  STATIC_ENDIAN_SELECT:1;    // 61 DMAH_STATIC_ENDIAN_SELECT
	Uint64  rsvd1:2;                   // 63:62 Reserved
}; 
union DMA_COMP_PARAMS_1_REG{
	Uint64  all;
	struct DMA_COMP_PARAMS_1_BITS bit;
};

//====================================================

struct DMA_COMPONENT_ID_BITS{
	
	Uint64  DMA_COMP_TYPE:32;     // 31:0 Designware Component Type number 0x44_57_11_10
	Uint64  DMA_COMP_VERSION:32;  // 63:32 Version of the component

};
union DMA_COMPONENT_ID_REG{
	Uint64  all;
	struct DMA_COMPONENT_ID_BITS bit;
};

//====================================================

struct DMA_REGS{
	struct CH_REGS      CH0;        //DMA Channel1 Registers
	struct CH_REGS      CH1;        //DMA Channel2 Registers
	struct CH_REGS      CH2;        //DMA Channel3 Registers
	struct CH_REGS      CH3;        //DMA Channel4 Registers

	Uint64 rsvd[44];      

	union RAW_REGS      RAWTFR;      // Raw Status for IntTfr Interrupt
	union RAW_REGS      RAWBLOCK;    // Raw Status for IntBlock Interrupt
	union RAW_REGS      RAWSRCTRAN;  // Raw Status for IntSrcTran Interrup
	union RAW_REGS      RAWDSTTRAN;  // Raw Status for IntDstTran Interrupt
	union RAW_REGS      RAWERR;      // Raw Status for IntErr Interrupt

	union STATUS_REGS   STATUSTFR;   // Status for IntTfr Interrupt
	union STATUS_REGS   STATUSBLOCK; // Status for IntBlock Interrupt
	union STATUS_REGS   STATUSSRCTRAN;  // Status for IntSrcTran Interrupt
	union STATUS_REGS   STATUSDSTTRAN;  // Status for IntDstTran Interrupt
	union STATUS_REGS   STATUSERR;      // Status for IntErr Interrupt

	union MASK_REGS     MASKTFR;    // Mask for IntTfr Interrupt
	union MASK_REGS     MASKBLOCK;    // Mask for IntBlock Interrupt
	union MASK_REGS     MASKSRCTRAN;    // Mask for IntSrcTran Interrupt
	union MASK_REGS     MASKDSTTRAN;    // Mask for IntDstTran Interrupt
	union MASK_REGS     MASKERR;    //   Mask for IntErr Interrupt

	union CLEAR_REGS    CLEARTFR;   //  Clear for IntTfr Interrupt
	union CLEAR_REGS    CLEARBLOCK;    //  Clear for IntBlock Interrupt
	union CLEAR_REGS    CLEARSRCTRAN;   // Clear for IntSrcTran Interrupt
	union CLEAR_REGS    CLEARDSTTRAN;    // Clear for IntDstTran Interrupt
	union CLEAR_REGS    CLEARERR;      //  Clear for IntErr Interrupt

	union STATUSINT_REG      STATUSINT;    // Status for each interrupt type

	union REQSRCREG_REG      REQSRCREG;    // Source Software Transaction Request Register,DMA is the flow controller,trigger transaction
	union REQDSTREG_REG      REQDSTREG;    // Destination Software Transaction Request Register,DMA is the flow controller,trigger transaction
	union SGLREQSRCREG_REG   SGLREQSRCREG;  // Single Source Transaction Request Register,DMA is the flow controller,trigger single transaction
	union SGLREQDSTREG_REG   SGLREQDSTREG;  // Single Destination Transaction Request Register,DMA is the flow controller,trigger single transaction
	//union LSTSRCREG_REG      LSTSRCREG;     // Last Source Transaction Request Register, ONLY when the peripheral is the flow controller
	//union LSTDSTREG_REG      LSTDSTREG;     // Last Destination Transaction Request Register, ONLY when the peripheral is the flow controller
	Uint64 rsvd1[2];

	union DMACFGREG_REG      DMACFGREG;    // DMA Configuration Register
	union CHENREG_REG        CHENREG;      // DMA Channel Enable Register
	union DMAIDREG_REG       DMAIDREG;     // DMA ID Register
	union DMATESTREG_REG     DMATESTREG;   // DMA Test Register   

	struct INTCTL_REGS       INTCTL0;      
	struct INTCTL_REGS       INTCTL1;
	struct INTCTL_REGS       INTCTL2;
	struct INTCTL_REGS       INTCTL3;

	union  DMA_COMP_PARAMS_6_REG    DMA_COMP_PARAMS_6;    //DW_ahb_dmac Component Parameters Register 6(Channel 7)
	union  DMA_COMP_PARAMS_5_REG    DMA_COMP_PARAMS_5;    //DW_ahb_dmac Component Parameters Register 5(Channel 5&6)
	union  DMA_COMP_PARAMS_4_REG    DMA_COMP_PARAMS_4;    //DW_ahb_dmac Component Parameters Register 4(Channel 3&4)
	union  DMA_COMP_PARAMS_3_REG    DMA_COMP_PARAMS_3;    //DW_ahb_dmac Component Parameters Register 3(Channel 1&2)
	union  DMA_COMP_PARAMS_2_REG    DMA_COMP_PARAMS_2;    //DW_ahb_dmac Component Parameters Register 2(Channel 0 )
	union  DMA_COMP_PARAMS_1_REG    DMA_COMP_PARAMS_1;    //DW_ahb_dmac Component Parameters Register 1
	union  DMA_COMPONENT_ID_REG     DMA_COMPONENT_ID;    //DMA Component ID Register

};


#endif

#ifdef  DSC28036


struct MODE_BITS {      // bits   description
    Uint32 PERINTSEL:5; // 4:0 Peripheral Interrupt and Sync Select Bits (R/W):
                        //        0     no interrupt
                        //        1     SEQ1INT & ADCSYNC
                        //        2     SEQ2INT
                        //        3     XINT1
                        //        4     XINT2
                        //        5     XINT3
                        //        6     XINT4
                        //        7     XINT5
                        //        8     XINT6
                        //        9     XINT7
                        //        10    XINT13
                        //        11    TINT0
                        //        12    TINT1
                        //        13    TINT2
                        //        14    MXEVTA & MXSYNCA
                        //        15    MREVTA & MRSYNCA
                        //        16    MXEVTB & MXSYNCB
                        //        17    MREVTB & MRSYNCB
                        //        18    ePWM1SOCA
                        //        19    ePWM1SOCB
                        //        20    ePWM2SOCA
                        //        21    ePWM2SOCB
                        //        22    ePWM3SOCA
                        //        23    ePWM3SOCB
                        //        24    ePWM4SOCA
                        //        25    ePWM4SOCB
                        //        26    ePWM5SOCA
                        //        27    ePWM5SOCB
                        //        28    ePWM6SOCA
                        //        29    ePWM6SOCB
                        //        30:31 no interrupt
    Uint32 rsvd1:2;     // 6:5    reserved
    Uint32 OVRINTE:1;   // 7      Overflow Interrupt Enable (R/W):
                        //        0     overflow interrupt disabled
                        //        1     overflow interrupt enabled
    Uint32 PERINTE:1;   // 8      Peripheral Interrupt Enable Bit (R/W):
                        //        0     peripheral interrupt disabled
                        //        1     peripheral interrupt enabled
    Uint32 CHINTMODE:1; // 9      Channel Interrupt Mode Bit (R/W):
                        //        0     generate interrupt at beginning of new
                        //              transfer
                        //        1     generate interrupt at end of transfer
    Uint32 ONESHOT:1;   // 10     One Shot Mode Bit (R/W):
                        //        0     only interrupt event triggers single
                        //              burst transfer
                        //        1     first interrupt triggers burst,
                        //              continue until transfer count is zero
    Uint32 CONTINUOUS:1;// 11     Continous Mode Bit (R/W):
                        //        0     stop when transfer count is zero
                        //        1     re-initialize when transfer count is
                        //              zero
    Uint32 SYNCE:1;     // 12     Sync Enable Bit (R/W):
                        //        0     ignore selected interrupt sync signal
                        //        1     enable selected interrupt sync signal
    Uint32 SYNCSEL:1;   // 13     Sync Select Bit (R/W):
                        //        0     sync signal controls source wrap
                        //              counter
                        //        1     sync signal controls destination wrap
                        //              counter
    Uint32 DATASIZE:1;  // 14     Data Size Mode Bit (R/W):
                        //        0     16-bit data transfer size
                        //        1     32-bit data transfer size
    Uint32 CHINTE:1;    // 15     Channel Interrupt Enable Bit (R/W):
                        //        0     channel interrupt disabled
                        //        1     channel interrupt enabled
    Uint32 rsvd2:16;   // 31:16 reserved
};

union MODE_REG {
    Uint32                all;
    struct MODE_BITS      bit;
};

//
// Channel CONTROL register bit definitions
//
struct CONTROL_BITS {       // bits   description
    Uint32 RUN:1;           // 0      Run Bit (R=0/W=1)
    Uint32 HALT:1;          // 1      Halt Bit (R=0/W=1)
    Uint32 SOFTRESET:1;     // 2      Soft Reset Bit (R=0/W=1)
    Uint32 PERINTFRC:1;     // 3      Interrupt Force Bit (R=0/W=1)
    Uint32 PERINTCLR:1;     // 4      Interrupt Clear Bit (R=0/W=1)
    Uint32 SYNCFRC:1;       // 5      Sync Force Bit (R=0/W=1)
    Uint32 SYNCCLR:1;       // 6      Sync Clear Bit (R=0/W=1)
    Uint32 ERRCLR:1;        // 7      Error Clear Bit (R=0/W=1)
    Uint32 PERINTFLG:1;     // 8      Interrupt Flag Bit (R):
                            //        0     no interrupt pending
                            //        1     interrupt pending
    Uint32 SYNCFLG:1;       // 9      Sync Flag Bit (R):
                            //        0     no sync pending
                            //        1     sync pending
    Uint32 SYNCERR:1;       // 10     Sync Error Flag Bit (R):
                            //        0     no sync error
                            //        1     sync error detected
    Uint32 TRANSFERSTS:1;   // 11     Transfer Status Bit (R):
                            //        0     no transfer in progress or pending
                            //        1     transfer in progress or pending
    Uint32 BURSTSTS:1;      // 12     Burst Status Bit (R):
                            //        0     no burst in progress or pending
                            //        1     burst in progress or pending
    Uint32 RUNSTS:1;        // 13     Run Status Bit (R):
                            //        0     channel not running or halted
                            //        1     channel running
    Uint32 OVRFLG:1;        // 14     Overflow Flag Bit(R)
                            //        0     no overflow event
                            //        1     overflow event
    Uint32 rsvd1:17;         // 31:15    reserved
};

union CONTROL_REG {
    Uint32                 all;
    struct CONTROL_BITS    bit;
};


//
// Burst Size
//
struct BURST_SIZE_BITS {   // bits  description
    Uint32 BURSTSIZE:5;    // 4:0   Burst transfer size
    Uint32 rsvd1:27;       // 31:5  reserved
};

union BURST_SIZE_REG {
    Uint32                  all;
    struct BURST_SIZE_BITS  bit;
};

//
// Burst Count
//
struct BURST_COUNT_BITS { // bits  description
    Uint32 BURSTCOUNT:5;   // 4:0   Burst transfer size
    Uint32 rsvd1:27;       // 31:5  reserved
};

union BURST_COUNT_REG {
    Uint32                   all;
    struct BURST_COUNT_BITS  bit;
};


//
// DMACTRL register bit definitions
//
struct DMACTRL_BITS {         // bits   description
    Uint32 HARDRESET:1;       // 0      Hard Reset Bit (R=0/W=1)
    Uint32 PRIORITYRESET:1;   // 1      Priority Reset Bit (R=0/W=1)
    Uint32 rsvd1:30;          // 31:2     reserved
};

union DMACTRL_REG {
    Uint32                 all;
    struct DMACTRL_BITS    bit;
};

//
// DEBUGCTRL register bit definitions
//
struct DEBUGCTRL_BITS {     // bits   description
    Uint32 rsvd1:15;        // 14:0   (R=0:0)
    Uint32 FREE:1;          // 15     Debug Mode Bit (R/W):
                            //    0   halt after current read-write operation
                            //    1   continue running
    Uint32 rsvd2:16;
};

union DEBUGCTRL_REG {
    Uint32                 all;
    struct DEBUGCTRL_BITS  bit;
};
//
// REVSION register bit definitions
//
struct REVSION_BITS {     // bits   description
    Uint32 REV:8;          // 7:0
    Uint32 TEPY:8;         //15:8
    Uint32 rsvd1:16;       // 31:16  reserved
};

union REVSION_REG {
    Uint32                 all;
    struct REVSION_BITS  bit;
};

//
// PRIORITYCTRL1 register bit definitions
//
struct PRIORITYCTRL1_BITS { // bits   description
   Uint32 CH1PRIORITY:1;    //     0      Ch1 Priority Bit (R/W):
                            //     0     same priority as all other channels
                            //     1     highest priority channel
   Uint32 rsvd1:31;         // 31:1     reserved
};

union PRIORITYCTRL1_REG {
    Uint32                     all;
    struct PRIORITYCTRL1_BITS  bit;
};

//
// PRIORITYSTAT register bit definitions:
//
struct PRIORITYSTAT_BITS {     // bits   description
    Uint32 ACTIVESTS:3;        // 2:0    Active Channel Status Bits (R):
                               //        0,0,0  no channel active
                               //        0,0,1  Ch1 channel active
                               //        0,1,0  Ch2 channel active
                               //        0,1,1  Ch3 channel active
                               //        1,0,0  Ch4 channel active
                               //        1,0,1  Ch5 channel active
                               //        1,1,0  Ch6 channel active
    Uint32 rsvd1:1;            // 3      (R=0)
    Uint32 ACTIVESTS_SHADOW:3; // 6:4   Active Channel Status Shadow Bits (R):
                               // 0,0,0 no channel active & interrupted by Ch1
                               // 0,0,1 cannot occur
                               // 0,1,0 Ch2 was active and interrupted by Ch1
                               // 0,1,1 Ch3 was active and interrupted by Ch1
                               // 1,0,0 Ch4 was active and interrupted by Ch1
                               // 1,0,1 Ch5 was active and interrupted by Ch1
                               // 1,1,0 Ch6 was active and interrupted by Ch1
    Uint32 rsvd2:25;            // 31:7     reserved
};

union PRIORITYSTAT_REG {
    Uint32                     all;
    struct PRIORITYSTAT_BITS   bit;
};




//
// DMA Channel Registers:
//
struct CH_REGS {
    union  MODE_REG            MODE;                 // Mode Register       00
    union  CONTROL_REG         CONTROL;              // Control Register    04
    union  BURST_SIZE_REG      BURST_SIZE;           // Burst Size Register   08
    union  BURST_COUNT_REG     BURST_COUNT;          // Burst Count Register  0C   
    int32                      SRC_BURST_STEP;       // Source Burst Step Register  10
    int32                      DST_BURST_STEP;       // Destination Burst Step Register  14
    Uint32                     TRANSFER_SIZE;        // Transfer Size Register   18
    Uint32                     TRANSFER_COUNT;       // Transfer Count Register  1c
    int32                      SRC_TRANSFER_STEP;    // Source Transfer Step Register  20
    int32                      DST_TRANSFER_STEP;    // Destination Transfer Step Register   24
    Uint32                     SRC_WRAP_SIZE;        // Source Wrap Size Register  28
    Uint32                     SRC_WRAP_COUNT;       // Source Wrap Count Register  2c
    int32                      SRC_WRAP_STEP;        // Source Wrap Step Register   30
    Uint32                     DST_WRAP_SIZE;        // Destination Wrap Size Register    34
    Uint32                     DST_WRAP_COUNT;       // Destination Wrap Count Register   38
    int32                      DST_WRAP_STEP;        // Destination Wrap Step Register    3C
    Uint32                     SRC_BEG_ADDR_SHADOW;  // Source Begin Address Shadow Register  40
    Uint32                     SRC_ADDR_SHADOW;      // Source Address Shadow Register  44
    Uint32                     SRC_BEG_ADDR_ACTIVE;  // Source Begin Address Active Register   48
    Uint32                     SRC_ADDR_ACTIVE;      // Source Address Active Register     4c
    Uint32                     DST_BEG_ADDR_SHADOW;  // Destination Begin Address Shadow Register   50
    Uint32                     DST_ADDR_SHADOW;      // Destination Address Shadow Register    54
    Uint32                     DST_BEG_ADDR_ACTIVE;  // Destination Begin Address Active Register   58
    Uint32                     DST_ADDR_ACTIVE;      // Destination Address Active Register   5c
    Uint32                     rsvd1[8];
};




struct DMA_REGS {
    union  DMACTRL_REG         DMACTRL;          // DMA Control Register    00
    union  DEBUGCTRL_REG       DEBUGCTRL;        // Debug Control Register  04
    union  REVSION_REG         REVSION;          //  REVSION Register  08
    union  PRIORITYCTRL1_REG   PRIORITYCTRL1;    // Priority Control 1 Register 0C
    union  PRIORITYSTAT_REG    PRIORITYSTAT;     // Priority Status Register  10
    Uint32                     rsvd3[27];        //14 - 7C
    struct CH_REGS             CH1;              // DMA Channel 1 Registers
    struct CH_REGS             CH2;              // DMA Channel 2 Registers
    struct CH_REGS             CH3;              // DMA Channel 3 Registers
    struct CH_REGS             CH4;              // DMA Channel 4 Registers
    struct CH_REGS             CH5;              // DMA Channel 5 Registers
    struct CH_REGS             CH6;              // DMA Channel 6 Registers
};








#endif




extern volatile  struct DMA_REGS *const  P_DmaRegs;
extern volatile  struct DMA_REGS DmaRegs;



#ifdef __cplusplus
}
#endif /* extern "C" */


#endif
