#ifndef PIDCTL_H_
#define PIDCTL_H_

#include <stdbool.h>

typedef struct pidctl_s
{
    float target;
    float kp;
    float ki;
    float kd;
    float ymin;
    float ymax;
    float period_s;
    float error;

    float _acc;
    float _xlast;
    float _tlast_s;
} pidctl_t;

void  pidctl_prime(pidctl_t *p);
bool  pidctl_needs_update(pidctl_t *p);
float pidctl_compute(pidctl_t *p, float _x);

#endif /* PIDCTL_H_ */
