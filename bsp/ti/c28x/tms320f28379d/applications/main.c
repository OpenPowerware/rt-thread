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
 * 2023-05-27     YunjieGu     Motor Control Demo
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <rthw.h>
#include <rtthread.h>
#include <math.h>
#include "board.h"
#include "applications.h"

float ppair = 4;
float rpm = 6000;

static float speed_cmd = 0;
static int count = 0;
static abc_dq_t current;
static abc_dq_t voltage;
static pid_ctl_t pid_current_d;
static pid_ctl_t pid_current_q;
static pid_ctl_t pid_speed;

static void print_float(char *str, float y);

interrupt void main_isr(void)
{
    float angle_e = eqep_get_angle() * ppair;
    float speed_m = eqep_get_speed();
    float torque_cmd;

    if(!(count % 10))
    {
        /* speed pid control */
        pid_speed.u = speed_cmd - speed_m;
        PID_UPDATE(pid_speed);
        torque_cmd = pid_speed.y;
    }

    /* current measure and transformation */
    inv_get_current(&current);
    ABC2DQ(current,angle_e);

    /* current (torque) pid control */
    pid_current_d.u = 0.0 - current.d;
    pid_current_q.u = torque_cmd - current.q;
    PID_UPDATE(pid_current_d);
    PID_UPDATE(pid_current_q);

    /* voltage command inverse transformation */
    voltage.d = pid_current_d.y;
    voltage.q = pid_current_q.y;
    DQ2ABC(voltage,angle_e);

    /* inverter pwm control */
    inv_set_duty(&voltage);

    count = (count++) % 100;
}

int main(void)
{
    float fm0 = rpm/60;
    float fe0 = fm0*ppair;
    float Xpu = 0.50;
    float Lpu = Xpu/fe0;

    float kp_current = Lpu * 1000 ;
    float ki_current = kp_current *250*2*PI;
    float kp_speed = 1e-4;
    float ki_speed = kp_speed *2*2*PI;

    float Ts = 100e-6;

    PID_INIT(pid_current_d,Ts   , kp_current,ki_current,0.0, 2/__sqrt(3),-2/__sqrt(3), 0.0,0.0);
    PID_INIT(pid_current_q,Ts   , kp_current,ki_current,0.0, 2/__sqrt(3),-2/__sqrt(3), 0.0,0.0);
    PID_INIT(pid_speed    ,Ts*10, kp_speed  ,ki_speed  ,0.0, 1.0        ,-1.0        , 0.0,0.0);

    eqep_setup();
    inv_setup();
}

static void print_float(char *str, float y)
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

static int set_speed(int argc, char *argv[])
{
    float _speed_cmd;

    if(argc != 2) {
        rt_kprintf("Usage: set_speed <value>\n");
        return -1;
    }

    _speed_cmd = strtof(argv[1],NULL);
    _speed_cmd = __fmin(_speed_cmd,6000.0);
    _speed_cmd = __fmax(_speed_cmd,-6000.0);

    print_float("Speed command set to (RPM): ",_speed_cmd);

    speed_cmd = _speed_cmd/60; //RPM2HZ

    return 0;
}
MSH_CMD_EXPORT(set_speed, "Set speed command");

static int get_angle(int argc, char *argv[])
{
    float angle;

    angle = eqep_get_angle();
    print_float("Mech angle (pu) = ", angle);

    return 0;
}
MSH_CMD_EXPORT(get_angle, "Get angle feedback");

static int get_speed(int argc, char *argv[])
{
    float speed;

    speed = eqep_get_speed();
    print_float("Mech speed (RPM) = ", speed*60);

    return 0;
}
MSH_CMD_EXPORT(get_speed, "Get speed feedback");
