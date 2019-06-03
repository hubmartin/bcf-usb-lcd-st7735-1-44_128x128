
#include <smooth.h>

float smooth_map(float x, float in_min, float in_max, float out_min, float out_max)
{

    if (x < in_min)
    {
        x = in_min;
    }

    if (x > in_max)
    {
        x = in_max;
    }

    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void smooth_init(smooth_t *self, float pos_current)
{
    self->pos_end = pos_current;
    self->pos_start = pos_current;
    self->pos_current = pos_current;

    self->active = false;
    self->max_duration = 3000;
}

void smooth_start(smooth_t *self, float target)
{
    self->pos_start = smooth_get(self);
    self->pos_end = target;
    self->start_tick = bc_tick_get();
    self->end_tick = self->start_tick + ((self->max_duration * abs(self->pos_start - self->pos_end ) ) / 180);

    self->active = true;
}

float smooth_get(smooth_t *self)
{
    if (bc_tick_get() >= self->end_tick)
    {
        self->active = false;
    }

    if (!self->active)
    {
        return self->pos_end;
    }

    return smooth_map(bc_tick_get(), self->start_tick, self->end_tick, self->pos_start, self->pos_end);
}
