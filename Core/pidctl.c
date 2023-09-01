#include "pidctl.h"
#include "main.h"
#include <string.h>

#define CLAMP(x, min, max) ((x > max) ? max : ((x < min) ? min : x))

// This depends on your BSP
#define TICKS_PER_SECOND 1000.0f
#define TICK_FUNCTION (HAL_GetTick())

static float
_get_tick_s(void)
{
	return (float)TICK_FUNCTION / TICKS_PER_SECOND;
}

bool
pidctl_needs_update(pidctl_t *p)
{
	// Add fabs() if you can't miss the timer overflow
	return (_get_tick_s() - p->tlast_s) >= p->period_s ? true : false;
}

void
pidctl_prime(pidctl_t *p)
{
	p->tlast_s = _get_tick_s();
}

void
pidctl_compute(pidctl_t *p)
{
	float error;
	float d;
	float y;
	// Latch 'x' for this computation.
	float x = *p->x;

	error = *p->target - x;

	p->acc += p->ki * error;

	p->acc = CLAMP(p->acc, p->ymin, p->ymax);

	d = *p->x - p->xlast;

	y = p->kp * error + p->acc - p->kd * d;

	y = CLAMP(y, p->ymin, p->ymax);

	p->xlast = x;
	p->tlast_s = _get_tick_s();

}
