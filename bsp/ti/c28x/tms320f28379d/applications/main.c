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
#include <math.h>
#include "board.h"

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

typedef struct
{
    float a;
    float b;
    float c;
    float d;
    float q;
    float theta;
} abc_dq_t;

float torque_cmd;

extern float eqep_get_angle();
extern float eqep_get_speed();
extern float eqep_setup();

void print_float(char *str, float y)
{
    long x;


    rt_kprintf(str);

    if(y >= 0.0f)
    {
        x = (long)(y * 1000.0f);
        rt_kprintf("%d.%d%d%d\n", x/1000,(x%1000)/100,(x%100)/10,x%10);
    }
    else
    {
        x = (long)(-y * 1000.0f);
        rt_kprintf("-%d.%d%d%d\n", x/1000,(x%1000)/100,(x%100)/10,x%10);
    }
}

static int set_torque(int argc, char *argv[]) {

    if(argc != 2) {
        rt_kprintf("Usage: set_torque <value>\n");
        return -1;
    }

    torque_cmd = atof(argv[1]);
    print_float("Torque command set to: ",torque_cmd);

    return 0;
}
MSH_CMD_EXPORT(set_torque, "Set torque command (pu)");

static int get_angle(int argc, char *argv[]) {

    float angle;

    angle = eqep_get_angle();
    print_float("Angle (rad) = ", angle);

    return 0;
}
MSH_CMD_EXPORT(get_angle, "Get angle feedback (rad)");

static int get_speed(int argc, char *argv[]) {

    float angle;

    angle = eqep_get_angle();
    print_float("speed (rad/s) = ", angle);

    return 0;
}
MSH_CMD_EXPORT(get_speed, "Get speed feedback (rad/s)");


int main(void)
{
    unsigned long x=10000;

    eqep_setup();

    while(x)
    {
        x--;
    }
}

/*@}*/
