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
#include <stdio.h>
#include <rthw.h>
#include <rtthread.h>
#include <math.h>
#include "board.h"


#define ABC2DQ(x,theta) do { \
    (x).d = (2.0f/3.0f) * ((x).a * __cospuf32(theta) + \
                           (x).b * __cospuf32(theta-1.0f/3.0f) + \
                           (x).c * __cospuf32(theta+1.0f/3.0f)); \
    (x).q =-(2.0f/3.0f) * ((x).a * __sinpuf32(theta) + \
                           (x).b * __sinpuf32(theta-1.0f/3.0f) + \
                           (x).c * __sinpuf32(theta+1.0f/3.0f)); \
} while(0)

#define DQ2ABC(x,theta) do { \
    (x).a = (x).d * __cospuf32(theta) - (x).q * __sinpuf32((x).theta); \
    (x).b = (x).d * __cospuf32(theta - 1.0f/3.0f) - (x).q * __sinpuf32((x).theta - 1.0f/3.0f); \
    (x).c = (x).d * __cospuf32(theta + 1.0f/3.0f) - (x).q * __sinpuf32((x).theta + 1.0f/3.0f); \
} while(0)

#define PID_UPDATE(x) do { \
    (x).yp = (x).kp * (x).u; \
    (x).yi = (x).yi + (x).ki * (x).u; \
    (x).yd = (x).kd * ((x).u - (x).uh); \
    (x).uh = (x).u; \
    (x).yi = __fmin((x).yi,(x).y_max); \
    (x).yi = __fmax((x).yi,(x).y_min); \
    (x).y  = (x).yp + (x).yi + (x).yd; \
    (x).y = __fmin((x).y,(x).y_max); \
    (x).y = __fmax((x).y,(x).y_min); \
} while(0)

#define PID_INIT(x,kp,ki,kd,y_max,y_min,yi,uh) do { \
    (x).kp = (kp); \
    (x).ki = (ki); \
    (x).kd = (kd); \
    (x).y_max = (y_max); \
    (x).y_min = (y_min); \
    (x).yi = (yi); \
    (x).uh = (uh); \
} while(0)

typedef struct
{
    float a;
    float b;
    float c;
    float d;
    float q;
} abc_dq_t;

typedef struct
{
    float u;
    float y;
    float uh;
    float yp;
    float yi;
    float yd;
    float y_max;
    float y_min;
    float kp;
    float ki;
    float kd;
} pid_ctl_t;

float torque_cmd;

extern float eqep_get_angle();
extern float eqep_get_speed();
extern float eqep_setup();

void print_float(char *str, float y)
{
    /* do not use sprintf: it may cause problems */

    long x;

    rt_kprintf(str);

    if(fabsf(y) > 100000)
    {
        rt_kprintf("%d\n",(long)y);
    }
    else if(y >= 0.0f)
    {
        x = (long)(y * 10000.0f);
        rt_kprintf("%d.%d%d%d%d\n", x/10000,(x%10000)/1000,(x%1000)/100,(x%100)/10,x%10);
    }
    else
    {
        x = (long)(-y * 10000.0f);
        rt_kprintf("-%d.%d%d%d%d\n", x/10000,(x%10000)/1000,(x%1000)/100,(x%100)/10,x%10);
    }
}

static int set_torque(int argc, char *argv[]) {

    if(argc != 2) {
        rt_kprintf("Usage: set_torque <value>\n");
        return -1;
    }

    torque_cmd = strtof(argv[1],NULL);
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

    float speed;

    speed = eqep_get_speed();
    print_float("speed (rad/s) = ", speed);

    return 0;
}
MSH_CMD_EXPORT(get_speed, "Get speed feedback (rad/s)");

interrupt void adc_isr(void)
{
    abc_dq_t current;
    abc_dq_t voltage;

    current.a = 0.0f;
    current.b = 0.0f;
    current.c = 0.0f;
    ABC2DQ(current,0.0);

}


int main(void)
{

    abc_dq_t current;
    abc_dq_t voltage;

    eqep_setup();


    float angle = eqep_get_angle();

    current.a = 0.5;
    current.b = 0.1;
    current.c = -0.6;
    ABC2DQ(current,angle);
    ABC2DQ(voltage,angle);

}

/*@}*/
