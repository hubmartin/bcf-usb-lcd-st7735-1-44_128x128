

#ifndef _SMOOTH_H
#define _SMOOTH_H

#include <bc_tick.h>

typedef struct smooth_t
{
    float pos_start;
    float pos_end;
    float pos_current;
    float max_duration;

    int interval;
    bc_tick_t start_tick;
    bc_tick_t end_tick;
    bool active;
} smooth_t;


void smooth_init(smooth_t *self, float pos_current);

void smooth_start(smooth_t *self, float target);

float smooth_get(smooth_t *self);

#endif //_SMOOTH_H
