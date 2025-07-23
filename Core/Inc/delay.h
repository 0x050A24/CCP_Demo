#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>
#include "gd32f30x_timer.h"

static inline void delay_us(uint32_t us)
{
  uint32_t prescaler = TIMER_PSC(TIMER1) + 1;
  uint32_t timer_clk = SystemCoreClock / prescaler;
  uint32_t start = TIMER_CNT(TIMER1);
  uint32_t ticks = (uint32_t)((float)(us * timer_clk) / 1000000.0F);
  while ((TIMER_CNT(TIMER1) - start) < ticks);
}
#endif
