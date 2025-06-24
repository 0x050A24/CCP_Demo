#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>
#include "gd32f30x_timer.h"

static inline void delay_us(uint32_t us)
{
    uint32_t start = TIMER_CNT(TIMER1);
    uint32_t ticks = (us << 4) + (us << 2); // us * 20
    while ((uint32_t)(TIMER_CNT(TIMER1) - start) < ticks);
}
#endif