
#ifndef	  __HX_RV_DEV_H
#define   __HX_RV_DEV_H

  
#define MSTATUS_UIE         0x00000001
#define MSTATUS_SIE         0x00000002
#define MSTATUS_HIE         0x00000004
#define MSTATUS_MIE         0x00000008
#define MSTATUS_UPIE        0x00000010
#define MSTATUS_SPIE        0x00000020
#define MSTATUS_HPIE        0x00000040
#define MSTATUS_MPIE        0x00000080
#define MSTATUS_SPP         0x00000100
#define MSTATUS_HPP         0x00000600
#define MSTATUS_MPP         0x00001800
#define MSTATUS_FS          0x00006000
#define MSTATUS_XS          0x00018000
#define MSTATUS_MPRV        0x00020000
#define MSTATUS_SUM         0x00040000
#define MSTATUS_MXR         0x00080000
#define MSTATUS_TVM         0x00100000
#define MSTATUS_TW          0x00200000
#define MSTATUS_TSR         0x00400000
#define MSTATUS32_SD        0x80000000
#define MSTATUS_UXL         0x0000000300000000
#define MSTATUS_SXL         0x0000000C00000000
#define MSTATUS64_SD        0x8000000000000000
  
#define SSTATUS_UIE         0x00000001
#define SSTATUS_SIE         0x00000002
#define SSTATUS_UPIE        0x00000010
#define SSTATUS_SPIE        0x00000020
#define SSTATUS_SPP         0x00000100
#define SSTATUS_FS          0x00006000
#define SSTATUS_XS          0x00018000
#define SSTATUS_SUM         0x00040000
#define SSTATUS_MXR         0x00080000
#define SSTATUS32_SD        0x80000000
#define SSTATUS_UXL         0x0000000300000000
#define SSTATUS64_SD        0x8000000000000000
  
#define DCSR_XDEBUGVER      (3U<<30)
#define DCSR_NDRESET        (1<<29)
#define DCSR_FULLRESET      (1<<28)
#define DCSR_EBREAKM        (1<<15)
#define DCSR_EBREAKH        (1<<14)
#define DCSR_EBREAKS        (1<<13)
#define DCSR_EBREAKU        (1<<12)
#define DCSR_STOPCYCLE      (1<<10)
#define DCSR_STOPTIME       (1<<9)
#define DCSR_CAUSE          (7<<6)
#define DCSR_DEBUGINT       (1<<5)
#define DCSR_HALT           (1<<3)
#define DCSR_STEP           (1<<2)
#define DCSR_PRV            (3<<0)
  
#define DCSR_CAUSE_NONE     0
#define DCSR_CAUSE_SWBP     1
#define DCSR_CAUSE_HWBP     2
#define DCSR_CAUSE_DEBUGINT 3
#define DCSR_CAUSE_STEP     4
#define DCSR_CAUSE_HALT     5
  
#define MCONTROL_TYPE(xlen)    (0xfULL<<((xlen)-4))
#define MCONTROL_DMODE(xlen)   (1ULL<<((xlen)-5))
#define MCONTROL_MASKMAX(xlen) (0x3fULL<<((xlen)-11))
  
#define MCONTROL_SELECT     (1<<19)
#define MCONTROL_TIMING     (1<<18)
#define MCONTROL_ACTION     (0x3f<<12)
#define MCONTROL_CHAIN      (1<<11)
#define MCONTROL_MATCH      (0xf<<7)
#define MCONTROL_M          (1<<6)
#define MCONTROL_H          (1<<5)
#define MCONTROL_S          (1<<4)
#define MCONTROL_U          (1<<3)
#define MCONTROL_EXECUTE    (1<<2)
#define MCONTROL_STORE      (1<<1)
#define MCONTROL_LOAD       (1<<0)
  
#define MCONTROL_TYPE_NONE      0
#define MCONTROL_TYPE_MATCH     2
  
#define MCONTROL_ACTION_DEBUG_EXCEPTION   0
#define MCONTROL_ACTION_DEBUG_MODE        1
#define MCONTROL_ACTION_TRACE_START       2
#define MCONTROL_ACTION_TRACE_STOP        3
#define MCONTROL_ACTION_TRACE_EMIT        4
  
#define MCONTROL_MATCH_EQUAL     0
#define MCONTROL_MATCH_NAPOT     1
#define MCONTROL_MATCH_GE        2
#define MCONTROL_MATCH_LT        3
#define MCONTROL_MATCH_MASK_LOW  4
#define MCONTROL_MATCH_MASK_HIGH 5
  
#define MIP_SSIP            (1 << IRQ_S_SOFT)
#define MIP_HSIP            (1 << IRQ_H_SOFT)
#define MIP_MSIP            (1 << IRQ_M_SOFT)
#define MIP_STIP            (1 << IRQ_S_TIMER)
#define MIP_HTIP            (1 << IRQ_H_TIMER)
#define MIP_MTIP            (1 << IRQ_M_TIMER)
#define MIP_SEIP            (1 << IRQ_S_EXT)
#define MIP_HEIP            (1 << IRQ_H_EXT)
#define MIP_MEIP            (1 << IRQ_M_EXT)
  
#define SIP_SSIP MIP_SSIP
#define SIP_STIP MIP_STIP
  
#define PRV_U 0
#define PRV_S 1
#define PRV_H 2
#define PRV_M 3
  
#define SATP32_MODE 0x80000000
#define SATP32_ASID 0x7FC00000
#define SATP32_PPN  0x003FFFFF
#define SATP64_MODE 0xF000000000000000
#define SATP64_ASID 0x0FFFF00000000000
#define SATP64_PPN  0x00000FFFFFFFFFFF
  
#define SATP_MODE_OFF  0
#define SATP_MODE_SV32 1
#define SATP_MODE_SV39 8
#define SATP_MODE_SV48 9
#define SATP_MODE_SV57 10
#define SATP_MODE_SV64 11
  
#define PMP_R     0x01
#define PMP_W     0x02
#define PMP_X     0x04
#define PMP_A     0x18
#define PMP_L     0x80
#define PMP_SHIFT 2
  
#define PMP_TOR   0x08
#define PMP_NA4   0x10
#define PMP_NAPOT 0x18
  
#define IRQ_S_SOFT   1
#define IRQ_H_SOFT   2
#define IRQ_M_SOFT   3
#define IRQ_S_TIMER  5
#define IRQ_H_TIMER  6
#define IRQ_M_TIMER  7
#define IRQ_S_EXT    9
#define IRQ_H_EXT    10
#define IRQ_M_EXT    11
#define IRQ_COP      12
#define IRQ_HOST     13
  
#define DEFAULT_RSTVEC     0x00001000
#define CLINT_BASE         0x02000000
#define CLINT_SIZE         0x000c0000
#define EXT_IO_BASE        0x40000000
#define DRAM_BASE          0x80000000
  
  // page table entry (PTE) fields
#define PTE_V     0x001 // Valid
#define PTE_R     0x002 // Read
#define PTE_W     0x004 // Write
#define PTE_X     0x008 // Execute
#define PTE_U     0x010 // User
#define PTE_G     0x020 // Global
#define PTE_A     0x040 // Accessed
#define PTE_D     0x080 // Dirty
#define PTE_SOFT  0x300 // Reserved for Software
  
#define PTE_PPN_SHIFT 10
  
#define PTE_TABLE(PTE) (((PTE) & (PTE_V | PTE_R | PTE_W | PTE_X)) == PTE_V)
  

  
#if __riscv_xlen == 64
# define MSTATUS_SD MSTATUS64_SD
# define SSTATUS_SD SSTATUS64_SD
# define RISCV_PGLEVEL_BITS 9
# define SATP_MODE SATP64_MODE
#else
# define MSTATUS_SD MSTATUS32_SD
# define SSTATUS_SD SSTATUS32_SD
# define RISCV_PGLEVEL_BITS 10
# define SATP_MODE SATP32_MODE
#endif
#define RISCV_PGSHIFT 12
#define RISCV_PGSIZE (1 << RISCV_PGSHIFT)

#define __REGNAME_TO_REGNU(reg)  #reg


#define read_csr(reg) ({ unsigned long __tmp; \
	asm volatile ("csrr %0, " __REGNAME_TO_REGNU(reg) : "=r"(__tmp)); \
	__tmp; })
  
#define write_csr(reg, val) ({ \
	asm volatile ("csrw " __REGNAME_TO_REGNU(reg) ", %0" :: "rK"(val)); })
  
#define swap_csr(reg, val) ({ unsigned long __tmp; \
	asm volatile ("csrrw %0, " __REGNAME_TO_REGNU(reg) ", %1" : "=r"(__tmp) : "rK"(val)); \
	__tmp; })
  
#define set_csr(reg, bit) ({ unsigned long __tmp; \
	asm volatile ("csrrs %0, " __REGNAME_TO_REGNU(reg) ", %1" : "=r"(__tmp) : "rK"(bit)); \
	__tmp; })
  
#define clear_csr(reg, bit) ({ unsigned long __tmp; \
	asm volatile ("csrrc %0, " __REGNAME_TO_REGNU(reg) ", %1" : "=r"(__tmp) : "rK"(bit)); \
	__tmp; })
  
#define rdtime() read_csr(time)
#define rdcycle() read_csr(cycle)
#define rdinstret() read_csr(instret)
 

#define __CSR_FFLAGS 0x1
#define __CSR_FRM 0x2
#define __CSR_FCSR 0x3
#define __CSR_CYCLE 0xc00
#define __CSR_TIME 0xc01
#define __CSR_INSTRET 0xc02
#define __CSR_HPMCOUNTER3 0xc03
#define __CSR_HPMCOUNTER4 0xc04
#define __CSR_HPMCOUNTER5 0xc05
#define __CSR_HPMCOUNTER6 0xc06
#define __CSR_HPMCOUNTER7 0xc07
#define __CSR_HPMCOUNTER8 0xc08
#define __CSR_HPMCOUNTER9 0xc09
#define __CSR_HPMCOUNTER10 0xc0a
#define __CSR_HPMCOUNTER11 0xc0b
#define __CSR_HPMCOUNTER12 0xc0c
#define __CSR_HPMCOUNTER13 0xc0d
#define __CSR_HPMCOUNTER14 0xc0e
#define __CSR_HPMCOUNTER15 0xc0f
#define __CSR_HPMCOUNTER16 0xc10
#define __CSR_HPMCOUNTER17 0xc11
#define __CSR_HPMCOUNTER18 0xc12
#define __CSR_HPMCOUNTER19 0xc13
#define __CSR_HPMCOUNTER20 0xc14
#define __CSR_HPMCOUNTER21 0xc15
#define __CSR_HPMCOUNTER22 0xc16
#define __CSR_HPMCOUNTER23 0xc17
#define __CSR_HPMCOUNTER24 0xc18
#define __CSR_HPMCOUNTER25 0xc19
#define __CSR_HPMCOUNTER26 0xc1a
#define __CSR_HPMCOUNTER27 0xc1b
#define __CSR_HPMCOUNTER28 0xc1c
#define __CSR_HPMCOUNTER29 0xc1d
#define __CSR_HPMCOUNTER30 0xc1e
#define __CSR_HPMCOUNTER31 0xc1f
#define __CSR_SSTATUS 0x100
#define __CSR_SIE 0x104
#define __CSR_STVEC 0x105
#define __CSR_SCOUNTEREN 0x106
#define __CSR_SSCRATCH 0x140
#define __CSR_SEPC 0x141
#define __CSR_SCAUSE 0x142
#define __CSR_STVAL 0x143
#define __CSR_SIP 0x144
#define __CSR_SATP 0x180
#define __CSR_MSTATUS 0x300
#define __CSR_MISA 0x301
#define __CSR_MEDELEG 0x302
#define __CSR_MIDELEG 0x303
#define __CSR_MIE 0x304
#define __CSR_MTVEC 0x305
#define __CSR_MCOUNTEREN 0x306
#define __CSR_MSCRATCH 0x340
#define __CSR_MEPC 0x341
#define __CSR_MCAUSE 0x342
#define __CSR_MTVAL 0x343
#define __CSR_MIP 0x344
#define __CSR_PMPCFG0 0x3a0
#define __CSR_PMPCFG1 0x3a1
#define __CSR_PMPCFG2 0x3a2
#define __CSR_PMPCFG3 0x3a3
#define __CSR_PMPADDR0 0x3b0
#define __CSR_PMPADDR1 0x3b1
#define __CSR_PMPADDR2 0x3b2
#define __CSR_PMPADDR3 0x3b3
#define __CSR_PMPADDR4 0x3b4
#define __CSR_PMPADDR5 0x3b5
#define __CSR_PMPADDR6 0x3b6
#define __CSR_PMPADDR7 0x3b7
#define __CSR_PMPADDR8 0x3b8
#define __CSR_PMPADDR9 0x3b9
#define __CSR_PMPADDR10 0x3ba
#define __CSR_PMPADDR11 0x3bb
#define __CSR_PMPADDR12 0x3bc
#define __CSR_PMPADDR13 0x3bd
#define __CSR_PMPADDR14 0x3be
#define __CSR_PMPADDR15 0x3bf
#define __CSR_TSELECT 0x7a0
#define __CSR_TDATA1 0x7a1
#define __CSR_TDATA2 0x7a2
#define __CSR_TDATA3 0x7a3
#define __CSR_DCSR 0x7b0
#define __CSR_DPC 0x7b1
#define __CSR_DSCRATCH 0x7b2
#define __CSR_LOOP	0x7c0 
#define __CSR_EALLOW 0x7c1 
#define __CSR_MCYCLE 0xb00
#define __CSR_MINSTRET 0xb02
#define __CSR_MHPMCOUNTER3 0xb03
#define __CSR_MHPMCOUNTER4 0xb04
#define __CSR_MHPMCOUNTER5 0xb05
#define __CSR_MHPMCOUNTER6 0xb06
#define __CSR_MHPMCOUNTER7 0xb07
#define __CSR_MHPMCOUNTER8 0xb08
#define __CSR_MHPMCOUNTER9 0xb09
#define __CSR_MHPMCOUNTER10 0xb0a
#define __CSR_MHPMCOUNTER11 0xb0b
#define __CSR_MHPMCOUNTER12 0xb0c
#define __CSR_MHPMCOUNTER13 0xb0d
#define __CSR_MHPMCOUNTER14 0xb0e
#define __CSR_MHPMCOUNTER15 0xb0f
#define __CSR_MHPMCOUNTER16 0xb10
#define __CSR_MHPMCOUNTER17 0xb11
#define __CSR_MHPMCOUNTER18 0xb12
#define __CSR_MHPMCOUNTER19 0xb13
#define __CSR_MHPMCOUNTER20 0xb14
#define __CSR_MHPMCOUNTER21 0xb15
#define __CSR_MHPMCOUNTER22 0xb16
#define __CSR_MHPMCOUNTER23 0xb17
#define __CSR_MHPMCOUNTER24 0xb18
#define __CSR_MHPMCOUNTER25 0xb19
#define __CSR_MHPMCOUNTER26 0xb1a
#define __CSR_MHPMCOUNTER27 0xb1b
#define __CSR_MHPMCOUNTER28 0xb1c
#define __CSR_MHPMCOUNTER29 0xb1d
#define __CSR_MHPMCOUNTER30 0xb1e
#define __CSR_MHPMCOUNTER31 0xb1f
#define __CSR_MHPMEVENT3 0x323
#define __CSR_MHPMEVENT4 0x324
#define __CSR_MHPMEVENT5 0x325
#define __CSR_MHPMEVENT6 0x326
#define __CSR_MHPMEVENT7 0x327
#define __CSR_MHPMEVENT8 0x328
#define __CSR_MHPMEVENT9 0x329
#define __CSR_MHPMEVENT10 0x32a
#define __CSR_MHPMEVENT11 0x32b
#define __CSR_MHPMEVENT12 0x32c
#define __CSR_MHPMEVENT13 0x32d
#define __CSR_MHPMEVENT14 0x32e
#define __CSR_MHPMEVENT15 0x32f
#define __CSR_MHPMEVENT16 0x330
#define __CSR_MHPMEVENT17 0x331
#define __CSR_MHPMEVENT18 0x332
#define __CSR_MHPMEVENT19 0x333
#define __CSR_MHPMEVENT20 0x334
#define __CSR_MHPMEVENT21 0x335
#define __CSR_MHPMEVENT22 0x336
#define __CSR_MHPMEVENT23 0x337
#define __CSR_MHPMEVENT24 0x338
#define __CSR_MHPMEVENT25 0x339
#define __CSR_MHPMEVENT26 0x33a
#define __CSR_MHPMEVENT27 0x33b
#define __CSR_MHPMEVENT28 0x33c
#define __CSR_MHPMEVENT29 0x33d
#define __CSR_MHPMEVENT30 0x33e
#define __CSR_MHPMEVENT31 0x33f
#define __CSR_MVENDORID 0xf11
#define __CSR_MARCHID 0xf12
#define __CSR_MIMPID 0xf13
#define __CSR_MHARTID 0xf14
#define __CSR_CYCLEH 0xc80
#define __CSR_TIMEH 0xc81
#define __CSR_INSTRETH 0xc82
#define __CSR_HPMCOUNTER3H 0xc83
#define __CSR_HPMCOUNTER4H 0xc84
#define __CSR_HPMCOUNTER5H 0xc85
#define __CSR_HPMCOUNTER6H 0xc86
#define __CSR_HPMCOUNTER7H 0xc87
#define __CSR_HPMCOUNTER8H 0xc88
#define __CSR_HPMCOUNTER9H 0xc89
#define __CSR_HPMCOUNTER10H 0xc8a
#define __CSR_HPMCOUNTER11H 0xc8b
#define __CSR_HPMCOUNTER12H 0xc8c
#define __CSR_HPMCOUNTER13H 0xc8d
#define __CSR_HPMCOUNTER14H 0xc8e
#define __CSR_HPMCOUNTER15H 0xc8f
#define __CSR_HPMCOUNTER16H 0xc90
#define __CSR_HPMCOUNTER17H 0xc91
#define __CSR_HPMCOUNTER18H 0xc92
#define __CSR_HPMCOUNTER19H 0xc93
#define __CSR_HPMCOUNTER20H 0xc94
#define __CSR_HPMCOUNTER21H 0xc95
#define __CSR_HPMCOUNTER22H 0xc96
#define __CSR_HPMCOUNTER23H 0xc97
#define __CSR_HPMCOUNTER24H 0xc98
#define __CSR_HPMCOUNTER25H 0xc99
#define __CSR_HPMCOUNTER26H 0xc9a
#define __CSR_HPMCOUNTER27H 0xc9b
#define __CSR_HPMCOUNTER28H 0xc9c
#define __CSR_HPMCOUNTER29H 0xc9d
#define __CSR_HPMCOUNTER30H 0xc9e
#define __CSR_HPMCOUNTER31H 0xc9f
#define __CSR_MCYCLEH 0xb80
#define __CSR_MINSTRETH 0xb82
#define __CSR_MHPMCOUNTER3H 0xb83
#define __CSR_MHPMCOUNTER4H 0xb84
#define __CSR_MHPMCOUNTER5H 0xb85
#define __CSR_MHPMCOUNTER6H 0xb86
#define __CSR_MHPMCOUNTER7H 0xb87
#define __CSR_MHPMCOUNTER8H 0xb88
#define __CSR_MHPMCOUNTER9H 0xb89
#define __CSR_MHPMCOUNTER10H 0xb8a
#define __CSR_MHPMCOUNTER11H 0xb8b
#define __CSR_MHPMCOUNTER12H 0xb8c
#define __CSR_MHPMCOUNTER13H 0xb8d
#define __CSR_MHPMCOUNTER14H 0xb8e
#define __CSR_MHPMCOUNTER15H 0xb8f
#define __CSR_MHPMCOUNTER16H 0xb90
#define __CSR_MHPMCOUNTER17H 0xb91
#define __CSR_MHPMCOUNTER18H 0xb92
#define __CSR_MHPMCOUNTER19H 0xb93
#define __CSR_MHPMCOUNTER20H 0xb94
#define __CSR_MHPMCOUNTER21H 0xb95
#define __CSR_MHPMCOUNTER22H 0xb96
#define __CSR_MHPMCOUNTER23H 0xb97
#define __CSR_MHPMCOUNTER24H 0xb98
#define __CSR_MHPMCOUNTER25H 0xb99
#define __CSR_MHPMCOUNTER26H 0xb9a
#define __CSR_MHPMCOUNTER27H 0xb9b
#define __CSR_MHPMCOUNTER28H 0xb9c
#define __CSR_MHPMCOUNTER29H 0xb9d
#define __CSR_MHPMCOUNTER30H 0xb9e
#define __CSR_MHPMCOUNTER31H 0xb9f
#define CAUSE_MISALIGNED_FETCH 0x0
#define CAUSE_FETCH_ACCESS 0x1
#define CAUSE_ILLEGAL_INSTRUCTION 0x2
#define CAUSE_BREAKPOINT 0x3
#define CAUSE_MISALIGNED_LOAD 0x4
#define CAUSE_LOAD_ACCESS 0x5
#define CAUSE_MISALIGNED_STORE 0x6
#define CAUSE_STORE_ACCESS 0x7
#define CAUSE_USER_ECALL 0x8
#define CAUSE_SUPERVISOR_ECALL 0x9
#define CAUSE_HYPERVISOR_ECALL 0xa
#define CAUSE_MACHINE_ECALL 0xb
#define CAUSE_FETCH_PAGE_FAULT 0xc
#define CAUSE_LOAD_PAGE_FAULT 0xd
#define CAUSE_STORE_PAGE_FAULT 0xf 

#ifdef  __clang__
	
	#define _SAVE_IRQ_STATUS()   Uint32  __mstatus =   read_csr(__CSR_MSTATUS); Uint32  __mepc =   read_csr(__CSR_MEPC);  Uint32  __loop =   read_csr(__CSR_LOOP);  
	#define _RESTORE_IRQ_STATUS()   write_csr(__CSR_MSTATUS,__mstatus);  write_csr(__CSR_MEPC,__mepc);   write_csr(__CSR_LOOP,__loop); 

#else
	
	#define _SAVE_IRQ_STATUS()   ;
	#define _RESTORE_IRQ_STATUS()   ;
	
#endif

#define   NOP   asm volatile("nop")

#define EALLOW  asm("csrsi 0x7C1, 0x01")  // eallow  register id is  0x7C1 ,enable write spieacl register
#define	EDIS  	asm("csrci 0x7C1, 0x01")  // disable

//#define  ESTOP0 asm(" ESTOP0") //if in debug ,set 1 dsp stop,not in debug, just a nop
#define  ESTOP0 asm(" ebreak"); //send debug call

#define  EINT	asm("csrsi mstatus, 0x8") 
#define  DINT   asm("csrci mstatus, 0x8") 





#define  ERTM 

#define  DRTM 


//#define  IDLE 
#include "stdio.h"

#include "stdbool.h"




#endif



