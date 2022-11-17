/*
//###########################################################################################################
//
//FILE:    DSC28034_link_RAM.ld
//
//
//TITLE: RAM Project linker file
//
//       Keep in mind that L0 is protected by 
//       the code security module
//
//       If the chip is in the encrypted,it will not
//       be able to read L0 data.
//       When the debugger is linked and encrypted,
//       the program on L0 cannot run correctly
//
//
//       If you need to create larger memory blocks,you 
//       can combine successive saram memory blocks
//###########################################################################################################
//$Release Data:October 11 ,2021$
//
//###########################################################################################################
*/

OUTPUT_ARCH( "riscv" )
ENTRY(_start)


MEMORY
{
  /*PAGE 0:*/
  PRAMM0 (rwx)  : ORIGIN = 0x0      , LENGTH = 0x800                                               /* on-chip RAM block M0 */ 
  PRAMM1 (rwx)  : ORIGIN = 0x800    , LENGTH = 0x800                                              /* on-chip RAM block M1*/

  AdcRegs_FILE (rwx)  :  ORIGIN  = 0x00001400 , LENGTH = 156                                   /* ADC registers */

  AdcResult_FILE (rwx)  :  ORIGIN  = 0x0000149c , LENGTH = 64                                   /* ADC result registers */

  CpuTimer0Regs_FILE  (rwx) : ORIGIN = 0x00001800 ,  LENGTH = 16                           /*  CPU timer0 registers  */
  CpuTimer1Regs_FILE  (rwx) : ORIGIN = 0x00001810 ,  LENGTH = 16                            /*  CPU timer1 registers */                        
  CpuTimer2Regs_FILE  (rwx) : ORIGIN = 0x00001820 ,  LENGTH = 16                           /*  CPU timer2 registers */

  PieCtrlRegs_FILE(rwx)  : ORIGIN  = 0x00001900 , LENGTH = 104                                 /*  PIE control registers */

  XIntruptRegs_FILE (rwx) :   ORIGIN  = 0x00001968 , LENGTH = 124                             /* XIntrupt registers */

  PieVectTable_FILE (rwx) :   ORIGIN  = 0x00001A00 , LENGTH = 448                            /* PIE PieVectTable  */

  PieEmuRegs_FILE (rwx)   :  ORIGIN = 0x1BC0, LENGTH = 4                                        /* Emu registers  */

  DmaRegs_FILE (rwx)      :  ORIGIN  = 0x00001C00 , LENGTH = 1024                           /* DMA registers */

  ECanaRegs_FILE (rwx)       :  ORIGIN  = 0x9000 , LENGTH = 104                              /* CAN registers */
  ECanaLAMRegs_FILE (rwx) 	 :  ORIGIN  = 0x9080 , LENGTH = 128                              /* CAN Local Acceptance Masks registers */ 
  ECanaMOTSRegs_FILE (rwx)   :  ORIGIN  = 0x9100 , LENGTH = 128                              /* CAN Message Object Time Stamps registers */                     
  ECanaMOTORegs_FILE (rwx)   :  ORIGIN  = 0x9180 , LENGTH = 128                              /* CAN Message Object Time-Out registers*/
  ECAN_MBOXES_FILE (rwx)   :    ORIGIN  = 0x9200 , LENGTH = 512                             /* CAN Mailbox registers*/
  
  EPwm1Regs_FILE (rwx)  :   ORIGIN  = 0xB000 , LENGTH = 0x100                                 /*  EPWM1 registers */  
  EPwm2Regs_FILE (rwx)  :   ORIGIN  = 0xB100 , LENGTH = 0x100                                 /*  EPWM2 registers */
  EPwm3Regs_FILE (rwx)  :   ORIGIN  = 0xB200 , LENGTH = 0x100                                 /*  EPWM3 registers */
  EPwm4Regs_FILE (rwx)  :   ORIGIN  = 0xB300 , LENGTH = 0x100                                 /*  EPWM4 registers */
  EPwm5Regs_FILE (rwx)  :   ORIGIN  = 0xB400 , LENGTH = 0x100                                 /*  EPWM5 registers */
  EPwm6Regs_FILE (rwx)  :   ORIGIN  = 0xB500 , LENGTH = 0x100                                 /*  EPWM6 registers */
  EPwm7Regs_FILE (rwx)  :   ORIGIN  = 0xB600 , LENGTH = 0x100                                 /*  EPWM7 registers */

  Comp1Regs_FILE  (rwx)   :  ORIGIN  = 0x00C000 , LENGTH = 40                                 /* COMP1 registers */
  Comp2Regs_FILE  (rwx)   :  ORIGIN  = 0x00C080 , LENGTH = 40                                 /* COMP2 registers */
  Comp3Regs_FILE  (rwx)   :  ORIGIN  = 0x00C100 , LENGTH = 40                                 /* COMP3 registers */

  ECap1Regs_FILE (rwx) : ORIGIN = 0xD000 ,  LENGTH = 48                                        /* ECAP1 registers */

  EQep1Regs_FILE (rwx)  : ORIGIN  =  0x0000D400 , LENGTH = 100                             /*  EQEP1 registers */
  EQep2Regs_FILE (rwx)  : ORIGIN  =  0x0000D480 , LENGTH = 100                             /*  EQEP2 registers */
  
  GpioCtrlRegs_FILE (rwx) : ORIGIN  = 0x0000D800 , LENGTH = 56                             /* GPIO control registers */                     
  GpioDataRegs_FILE (rwx) : ORIGIN  = 0x0000D838 , LENGTH = 48                             /* GPIO data registers */
  GpioIntRegs_FILE (rwx)  : ORIGIN  = 0x0000D868 , LENGTH = 16                             /*GPIO interrupt registers */

  SysCtrlRegs_FILE (rw)  : ORIGIN  = 0x0000DC00 , LENGTH = 0x100                           /* system control registers */


  SciaRegs_FILE (rwx)  :  ORIGIN  = 0x0000E000 , LENGTH = 248                                 /* SCI-A registers */

  I2caRegs_FILE (rwx)  :   ORIGIN  = 0x0000E400 , LENGTH = 256                                /* IIC-A registers */                        
  
  SpiaRegs_FILE (rwx)  :  ORIGIN  = 0x0000E800 , LENGTH = 48                                  /* SPI-A control registers */                                  
  SpibRegs_FILE (rwx)  :  ORIGIN  = 0x0000E880 , LENGTH = 48                                  /* SPI-B control registers */


  PRAML0 (rwx)  : ORIGIN = 0x10000  , LENGTH = 0x4000                                            /* on-chip RAM block L0 */
  PRAML1 (rwx)  : ORIGIN = 0x14000  , LENGTH = 0x4000                                            /* on-chip RAM block L1 */

  FlashRegs_FILE (rwx)    :  ORIGIN  = 0x7AF800 , LENGTH = 188                                  /* FLASH registers */
  
  
  FLASH  (rwx)  : ORIGIN = 0x700000 , LENGTH = 0x3FFE8                                          /* on-chip FLASH */ 

  CsmPwl_FILE (rwx)       :  ORIGIN  = 0x73FFF0 , LENGTH = 16                                     /* on-chip password area */

  LinaRegs_FILE (rwx)    :  ORIGIN  = 0xC400 , LENGTH = 0x400                                   /* lin registers */

  HRCap1Regs_FILE(rwx)   :  ORIGIN  = 0xCC00 , LENGTH = 0x70                                 /* HRCAP1 registers */  

  HRCap2Regs_FILE(rwx)   :  ORIGIN  = 0xCC80 , LENGTH = 0x70                                /* HRCAP2 registers */                              

  BootRom_FILE (rx)   :  ORIGIN  = 0x7F8000 , LENGTH = 0x8000                                /* on-chip Bootrom */

  Flash_Reset_FILE(rwx) : ORIGIN  = 0x73FFE8 , LENGTH = 8                                        /*FLASH reset address*/

  USER_OTP_FILE(rw)	: 		 ORIGIN  = 0x7A0000 , LENGTH = 0x2000                          /*on-chip OTP*/

  IER_REGISTER_FILE(rw) :  ORIGIN  = 0x100C10 , LENGTH = 0x04                                 /* IER registers */                           
  IFR_REGISTER_FILE(rw) :  ORIGIN  = 0x100D10 , LENGTH = 0x04                                 /* IFR registers */
}

/*----------------------------------------------------------------------*/
/* Sections                                                             */
/*----------------------------------------------------------------------*/

SECTIONS
{

  .AdcRegs(NOLOAD) :  { *(.AdcRegs) }  >  AdcRegs_FILE

  .AdcResult(NOLOAD) : { *(.AdcResult)  }  > AdcResult_FILE

  .ECap1Regs(NOLOAD) : { *(.ECap1Regs) }  > ECap1Regs_FILE

  .CpuTimer0Regs(NOLOAD) : { *(.CpuTimer0Regs) }	 >  CpuTimer0Regs_FILE
  .CpuTimer1Regs(NOLOAD) : { *(.CpuTimer1Regs) }	 >  CpuTimer1Regs_FILE
  .CpuTimer2Regs(NOLOAD) : { *(.CpuTimer2Regs) }	 >  CpuTimer2Regs_FILE


  .PieCtrlRegs(NOLOAD) : { *(.PieCtrlRegs)  }  > PieCtrlRegs_FILE

  .PieVectTable(NOLOAD) : { *(.PieVectTable)  } >  PieVectTable_FILE

  .XIntruptRegs(NOLOAD) :  { *(.XIntruptRegs) }  >  XIntruptRegs_FILE
  
  .PieEmuRegs(NOLOAD)    :  { *(.PieEmuRegs) } > PieEmuRegs_FILE 
  
  .EQep1Regs(NOLOAD)  : { *(.EQep1Regs) }  >  EQep1Regs_FILE
  .EQep2Regs(NOLOAD)  : { *(.EQep2Regs) }  >  EQep2Regs_FILE
  
  .SpiaRegs(NOLOAD) : { *(.SpiaRegs) } > SpiaRegs_FILE
  .SpibRegs(NOLOAD) : { *(.SpibRegs) } > SpibRegs_FILE

  .SciaRegs(NOLOAD) : { *(.SciaRegs) }  > SciaRegs_FILE

  .I2caRegs(NOLOAD) : { *(.I2caRegs) } > I2caRegs_FILE

  .EPwm1Regs(NOLOAD)  : { *(.EPwm1Regs) } >  EPwm1Regs_FILE
  .EPwm2Regs(NOLOAD)  : { *(.EPwm2Regs) } >  EPwm2Regs_FILE
  .EPwm3Regs(NOLOAD)  : { *(.EPwm3Regs) } >  EPwm3Regs_FILE
  .EPwm4Regs(NOLOAD)  : { *(.EPwm4Regs) } >  EPwm4Regs_FILE
  .EPwm5Regs(NOLOAD)  : { *(.EPwm5Regs) } >  EPwm5Regs_FILE
  .EPwm6Regs(NOLOAD)  : { *(.EPwm6Regs) } >  EPwm6Regs_FILE
  .EPwm7Regs(NOLOAD)  : { *(.EPwm7Regs) } >  EPwm7Regs_FILE

  .GpioCtrlRegs(NOLOAD)  :  { *(.GpioCtrlRegs) }  >  GpioCtrlRegs_FILE
  .GpioDataRegs(NOLOAD)  :  { *(.GpioDataRegs) }  > GpioDataRegs_FILE  
  .GpioIntRegs(NOLOAD)   :  { *(.GpioIntRegs) }  > GpioIntRegs_FILE

  .SysCtrlRegs(NOLOAD)   :  { *(.SysCtrlRegs)  }  > SysCtrlRegs_FILE
  
  .Comp1Regs(NOLOAD)     :  { *(.Comp1Regs)  }  >  Comp1Regs_FILE
  .Comp2Regs(NOLOAD)     :  { *(.Comp2Regs)  }  >  Comp2Regs_FILE

  .DmaRegs(NOLOAD)       :  { *(.DmaRegs)    }  >  DmaRegs_FILE

  .ECanaRegs(NOLOAD)    :  { *(.ECanaRegs) }  >  ECanaRegs_FILE
  .ECanaLAMRegs(NOLOAD) :  { *(.ECanaLAMRegs) }  >  ECanaLAMRegs_FILE
  .ECanaMOTSRegs(NOLOAD) : { *(.ECanaMOTSRegs) }  >  ECanaMOTSRegs_FILE
  .ECanaMOTORegs(NOLOAD) : { *(.ECanaMOTORegs) }  >  ECanaMOTORegs_FILE
  .ECAN_MBOXES(NOLOAD) :  { *(.ECAN_MBOXES) }  >  ECAN_MBOXES_FILE

  .LinaRegs(NOLOAD)       :  { *(.LinaRegs)    }  >  LinaRegs_FILE

  .FlashRegs(NOLOAD)     :  { *(.FlashRegs)  }  >  FlashRegs_FILE

  .CsmPwl(NOLOAD)        :  { *(.CsmPwl)  }   > CsmPwl_FILE


  .Flash_Reset(NOLOAD)    : { *(.Flash_Reset) }  >  Flash_Reset_FILE

  .HRCap1Regs(NOLOAD)  :  { *(.HRCap1Regs) } >   HRCap1Regs_FILE

  .HRCap2Regs(NOLOAD)  :  { *(.HRCap2Regs) } >   HRCap2Regs_FILE

  ._IQsinTable 0x00000000007f8000 (NOLOAD):  { IQsinTable = .  ;
  IQcosTable = . + 512 ;
  }  > BootRom_FILE
  ._IQdivRoundSatTable 0x00000000007f8a04 (NOLOAD):   { IQdivRoundSatTable = .  ; 
  }  > BootRom_FILE
  ._IQdivTable 0x00000000007f8a20 (NOLOAD):  { IQdivTable = . ; 
  }  > BootRom_FILE
  ._IQisqrtRoundSatTable 0x00000000007f8e24 (NOLOAD):  { IQisqrtRoundSatTable = .;
   }  > BootRom_FILE
  ._IQisqrtTable 0x00000000007f8e44 (NOLOAD) :  { IQisqrtTable = . ;
   }  > BootRom_FILE
  ._IQatan2HalfPITable 0x00000000007f9048 (NOLOAD):  { IQatan2HalfPITable = . ; 
  }  > BootRom_FILE
  ._IQatan2Table 0x00000000007f90c4 (NOLOAD) :  { IQatan2Table = . ; 
  }  > BootRom_FILE
  ._IQexpTableMinMax 0x00000000007f96a0 (NOLOAD):  { IQexpTableMinMax  = . ;
  }  > BootRom_FILE
  ._IQexpTableCoeff 0x00000000007f9790 (NOLOAD):  { IQexpTableCoeff = . ;
  }  > BootRom_FILE
  ._IQasinTable 0x00000000007f97b8 (NOLOAD):  { IQasinTable = . ;
   }  > BootRom_FILE
  
  .USER_OTP_TABLE(NOLOAD)   : {*(.User_Otp_Table)}  >  USER_OTP_FILE


  .IER_REGISTER(NOLOAD) : {*(.ier_register)} > IER_REGISTER_FILE
  .IFR_REGISTER(NOLOAD) : {*(.ifr_register)} > IFR_REGISTER_FILE
  
  /* text: test code section */
  . = 0x00014000;
  .text.init  : { *(.text.init)  }  > PRAML1
  
  .text ALIGN(0x80) : { *(.text) *(.IQmath)  *(.text*)  *(.rodata .rodata.*)  } > PRAML1
  
  /* data segment */

  .sdata   ALIGN(0x80)   :  {
    __data_start = .;
	__global_pointer$ = . + 0x800;
 	*(.data)  *(.data.*)
    *(.srodata.cst16) *(.srodata.cst8) *(.srodata.cst4) *(.srodata.cst2) *(.srodata*)
    *(.sdata .sdata.* .gnu.linkonce.s.*)
	*(ramfuncs)
	*(L1)
     __data_end = .;
  }  > PRAML1
 
  /* bss segment */
  
  .sbss ALIGN(0x80) : {
   __bss_start = .;
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)

	
  }  > PRAML1
  .bss ALIGN(0x80) : { *(.bss)  *(.bss.*)   __bss_end = .; } > PRAML1
 

  
  /* thread-local data segment */
  .tdata ALIGN(0x80) :
  {
    _tls_data = .;
    *(.tdata.begin)
    *(.tdata)
    *(.tdata.end)
  }  > PRAML1

  
  .tbss ALIGN(0x80) :
  {
    *(.tbss)
    *(.tbss.end)
  }  > PRAML1

  . = 0x00010000;
  .L0  :
  {
  __l0_data_start = . ;
  *(L0)
  __l0_data_end = . ;
  } > PRAML0
 
 
 . = 0x0;
  .M0  :
  {
  __m0_data_start = . ;
  __global_sp$ = . + 0x800;
  *(M0)
  __m0_data_end = . ;
  } > PRAMM0

  . = 0x800;
  .M1 : 
  {
  __m1_data_start = . ;
  *(ramconsts)
  *(M1)
  __m1_data_end = . ;
  } > PRAMM1

  

  
  /* End of uninitalized data segement */
  _end = .;
}

