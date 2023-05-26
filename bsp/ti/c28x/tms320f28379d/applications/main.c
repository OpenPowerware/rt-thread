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
#include <stdlib.h>
#include <rthw.h>
#include <rtthread.h>
#include "board.h"
#include <math.h>

typedef struct
{
    float a;
    float b;
    float c;
    float d;
    float q;
    float theta;
} abc_dq_t;

#define PI 3.14159265358979323846f

#define ABC2DQ(x) do { \
    (x)->d = 2.0f / 3.0f * ((x)->a * cosf((x)->theta) + \
                            (x)->b * cosf((x)->theta - 2.0f * PI / 3.0f) + \
                            (x)->c * cosf((x)->theta + 2.0f * PI / 3.0f)); \
    (x)->q =-2.0f / 3.0f * ((x)->a * sinf((x)->theta) + \
                            (x)->b * sinf((x)->theta - 2.0f * PI / 3.0f) + \
                            (x)->c * sinf((x)->theta + 2.0f * PI / 3.0f)); \
} while(0)

#define DQ2ABC(x) do { \
    (x)->a = (x)->d * cosf((x)->theta) - (x)->q * sinf((x)->theta); \
    (x)->b = (x)->d * cosf((x)->theta - 2.0f * PI / 3.0f) - (x)->q * sinf((x)->theta - 2.0f * PI / 3.0f); \
    (x)->c = (x)->d * cosf((x)->theta + 2.0f * PI / 3.0f) - (x)->q * sinf((x)->theta + 2.0f * PI / 3.0f); \
} while(0)

float torque_cmd;

static int set_torque(int argc, char *argv[]) {

    int x;

    if(argc != 2) {
        rt_kprintf("Usage: set_torque <value>\n");
        return -1;
    }

    torque_cmd = atof(argv[1]);

    if(torque_cmd >= 0.0f)
    {
        x = (int)(torque_cmd * 1000.0f);
        rt_kprintf("Torque command set to: %hd.%hd%hd%hd\n", x/1000,(x%1000)/100,(x%100)/10,x%10);
    }
    else
    {
        x = (int)(-torque_cmd * 1000.0f);
        rt_kprintf("Torque command set to: -%hd.%hd%hd%hd\n", x/1000,(x%1000)/100,(x%100)/10,x%10);
    }

    return 0;
}
MSH_CMD_EXPORT(set_torque, "Set the value of a global parameter");



int main(void)
{
    unsigned long x=10000;

    while(x)
    {
        x--;
    }
}

/*@}*/
