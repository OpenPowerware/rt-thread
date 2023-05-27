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
    (x).a = (x).d * __cospuf32(theta) - (x).q * __sinpuf32(theta); \
    (x).b = (x).d * __cospuf32(theta - 1.0f/3.0f) - (x).q * __sinpuf32(theta - 1.0f/3.0f); \
    (x).c = (x).d * __cospuf32(theta + 1.0f/3.0f) - (x).q * __sinpuf32(theta + 1.0f/3.0f); \
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

#define PID_INIT(x,_kp,_ki,_kd,_y_max,_y_min,_yi,_uh) do { \
    (x).kp = (_kp); \
    (x).ki = (_ki); \
    (x).kd = (_kd); \
    (x).y_max = (_y_max); \
    (x).y_min = (_y_min); \
    (x).yi = (_yi); \
    (x).uh = (_uh); \
} while(0)

float torque_cmd;

extern float eqep_get_angle();
extern float eqep_get_speed();
extern float eqep_setup();
extern void inv_set_duty(abc_dq_t *);
extern void inv_get_current(abc_dq_t *);

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
    torque_cmd = __fmin(torque_cmd,1.0);
    torque_cmd = __fmax(torque_cmd,-1.0);

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
}


int main(void)
{

    abc_dq_t current;
    abc_dq_t voltage;
    pid_ctl_t pid_id;
    pid_ctl_t pid_iq;

    eqep_setup();

    PID_INIT(pid_id,0.0,0.0,0.0,1.0,-1.0,0.0,0.0);
    PID_INIT(pid_iq,0.0,0.0,0.0,1.0,-1.0,0.0,0.0);

    float angle = eqep_get_angle();

    inv_get_current(&current);

    ABC2DQ(current,angle);

    pid_id.u = 0.0 - current.d;
    pid_iq.u = torque_cmd - current.q;

    PID_UPDATE(pid_id);
    PID_UPDATE(pid_iq);

    voltage.d = pid_id.y;
    voltage.q = pid_iq.y;

    DQ2ABC(voltage,angle);

    inv_set_duty(&voltage);
}

/*@}*/
