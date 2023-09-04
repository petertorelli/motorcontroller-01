#include "pidctl.h"
#include "main.h"
#include <string.h>

#define CLAMP(x, min, max) ((x > max) ? max : ((x < min) ? min : x))

// This depends on your BSP
#define TICKS_PER_SECOND 1000.0f
#define TICK_FUNCTION    (HAL_GetTick())

static float
_get_tick_s(void)
{
    return (float)TICK_FUNCTION / TICKS_PER_SECOND;
}

bool
pidctl_needs_update(pidctl_t *p)
{
    // Add fabs() if you can't miss the timer overflow
    return (_get_tick_s() - p->_tlast_s) >= p->period_s ? true : false;
}

void
pidctl_prime(pidctl_t *p)
{
    p->_tlast_s = _get_tick_s();
}

float
pidctl_compute(pidctl_t *p, float _x)
{
    float error;
    float d;
    float y;

    error       = p->target - _x;
    p->_acc     = p->_acc + p->ki * error;
    p->_acc     = CLAMP(p->_acc, p->ymin, p->ymax);
    d           = _x - p->_xlast;
    y           = (p->kp * error) + p->_acc - (p->kd * d);
    y           = CLAMP(y, p->ymin, p->ymax);
    p->_xlast   = _x;
    p->_tlast_s = _get_tick_s();

    return y;
}
