#ifndef PIDCTL_H_
#define PIDCTL_H_

#include <stdbool.h>

typedef struct pidctl_s
{
	// I prefer attaching control to the struct
	float *x;
	float *y;
	float *target;

	float kp;
	float ki;
	float kd;

	float ymin;
	float ymax;

	float acc;

	float xlast;
	float tlast_s;

	float period_s;

} pidctl_t;

void pidctl_prime(pidctl_t *p);
void pidctl_compute(pidctl_t *p);

#endif /* PIDCTL_H_ */
