/*
 * main.h
 *
 *  Created on: 27 May 2023
 *      Author: ygu2
 */

#ifndef APPLICATIONS_APPLICATIONS_H_
#define APPLICATIONS_APPLICATIONS_H_

#define MOD_INDEX 1.0

extern float eqep_get_angle();
extern float eqep_get_speed();
extern float eqep_setup(float);
extern void inv_set_duty(abc_dq_t *, float);
extern void inv_get_current(abc_dq_t *);
extern void inv_setup(float);

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

#define PID_INIT(x,_Ts,_kp,_ki,_kd,_y_max,_y_min,_yi,_uh) do { \
    (x).kp = (_kp); \
    (x).ki = (_ki*_Ts); \
    (x).kd = (_kd/_Ts); \
    (x).y_max = (_y_max); \
    (x).y_min = (_y_min); \
    (x).yi = (_yi); \
    (x).uh = (_uh); \
} while(0)


#endif /* APPLICATIONS_APPLICATIONS_H_ */
