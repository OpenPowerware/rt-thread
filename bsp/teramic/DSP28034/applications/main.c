/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2006-08-31     Bernard      first implementation
 * 2018-09-02     xuzhuoyi     modify for TMS320F28379D version
 * 2022-08-21     qiyu         modify the entry function
 */

#include <stdint.h>
#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include "DSP2803x_Device.h"
void thread1_entry(void* parameter)
{
	while(1)
	{
		rt_thread_yield();
	}
}

void thread2_entry(void* parameter)
{
	while(1)
	{
		rt_thread_yield();
	}
}

int main(void)
{
    volatile unsigned long x=100000;
    rt_thread_t thread1=NULL,thread2=NULL;

    rt_uint32_t para1=0,para2=0;

    thread1 = rt_thread_create("thread1",thread1_entry,&para1,400,6,10);
    thread2 = rt_thread_create("thread2",thread2_entry,&para2,400,6,10);

    rt_thread_startup(thread1);
    rt_thread_startup(thread2);

    while(x)
    {
        x--;
    }

    x = 100000000;
}
/*@}*/
