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
float Ts = 100e-6;
float speed_cmd = 20.0;
float torque_cmd = 0.0;
float speed_m;
float angle_e;
float angle_e_cal = 0.04;
float time = 0.0;
int count = 0;
abc_dq_t current;
abc_dq_t voltage;
pid_ctl_t pid_current_d;
pid_ctl_t pid_current_q;
pid_ctl_t pid_speed;

static void print_float(char *str, float y);

interrupt void main_isr(void)
{
    speed_m = eqep_get_speed();
    angle_e = eqep_get_angle() * ppair + angle_e_cal;

//    if(!(count % 10))
//    {
//        /* speed pid control */
//        pid_speed.u = speed_cmd - speed_m;
//        PID_UPDATE(pid_speed);
//        torque_cmd = pid_speed.y;
//    }

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
    inv_set_duty(&voltage, 100000000*Ts);

    /* update timer */
    count = (count++) % 100;
    time += Ts;

    EALLOW;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    EDIS;
}

int main(void)
{
    float fm0 = rpm/60;
    float fe0 = fm0*ppair;
    float Xpu = 0.1;
    float Lpu = Xpu/fe0;

    float kp_current = Lpu * 500;
    float ki_current = kp_current *100*2*PI;
    float kp_speed = 1e-4;
    float ki_speed = kp_speed *2*2*PI;

    PID_INIT(pid_current_d,Ts   , kp_current,ki_current,0.0, MOD_INDEX*2.0/__sqrt(3.0),-MOD_INDEX*2.0/__sqrt(3.0), 0.0,0.0);
    PID_INIT(pid_current_q,Ts   , kp_current,ki_current,0.0, MOD_INDEX*2.0/__sqrt(3.0),-MOD_INDEX*2.0/__sqrt(3.0), 0.0,0.0);
    PID_INIT(pid_speed    ,Ts*10, kp_speed  ,ki_speed  ,0.0, 0.0                      ,0.0                       , 0.0,0.0);

    eqep_setup(1e-3);
    inv_setup(Ts);
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

static int set_torque(int argc, char *argv[])
{
    float _torque_cmd;

    if(argc != 2) {
        rt_kprintf("Usage: set_torque <value>\n");
        return -1;
    }

    _torque_cmd = strtof(argv[1],NULL);
    _torque_cmd = __fmin(_torque_cmd,1.0);
    _torque_cmd = __fmax(_torque_cmd,-1.0);

    print_float("Torque command set to (pu): ",_torque_cmd);

    torque_cmd = _torque_cmd;

    return 0;
}
MSH_CMD_EXPORT(set_torque, "Set torque command");

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
