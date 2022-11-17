#include <rtthread.h>
#include "board.h"
#include "dsc_config.h"
#include <syscalls.h>
#include "drv_gpio.h"

extern int rt_hw_sci_init(void);
extern rt_base_t rt_hw_save_context(void);
extern void rt_hw_restore_context(rt_base_t);

__interrupt void cpu_timer2_isr(void)
{
	rt_base_t context;

	CpuTimer2Regs.TCR.all = 0xC000;

	context = rt_hw_save_context();

    rt_interrupt_enter();

    rt_tick_increase();

    rt_interrupt_leave();
    rt_hw_restore_context(context);
}

void rt_hw_board_init()
{
    /* Configure the system clock @ 84 Mhz */
    InitSysCtrl();

    DINT;
    InitPieCtrl();

    IER_DISABLE(0xffff); /*禁能所有中断*/
    IFR_DISABLE(0xffff); /*清所有中断标志位*/

    InitPieVectTable();

	EALLOW;
	PieCtrlRegs.PIEACK.all = 0xffff; /*清除所有中断*/
	EDIS;

    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TINT2 = &cpu_timer2_isr;
    EDIS;

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer2, 200, 1000000 / RT_TICK_PER_SECOND);
    CpuTimer2Regs.TCR.all = 0x4000;
    IER |= M_INT14;
    //IER |= M_INT1;

	//EINT;
	ERTM;
#ifdef RT_USING_HEAP
    rt_system_heap_init((void*)(0x14000), (void*)(0x18000));
#endif

#ifdef RT_USING_SERIAL
    rt_hw_sci_init();
#endif

#ifdef RT_USING_PIN
    rt_hw_pin_init();
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif
}

int super_main()
{
    /* _args_main is the entry point called by _c_int00. We define it
     * here to override the one defined by the compiler in args_main.c */

    extern int rtthread_startup();

    /* startup RT-Thread RTOS */
    rtthread_startup();
    /* never reach here*/
    return 0;
}
