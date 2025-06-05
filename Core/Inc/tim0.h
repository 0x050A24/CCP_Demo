#ifndef _TIM0_H_
#define _TIM0_H_

#include "gd32f30x.h"

void TIM0_PWM_Init(void);
uint8_t calculate_deadtime_value(uint32_t deadtime_ns, uint32_t timer_clk_hz);
#endif /* _TIM0_H_ */
