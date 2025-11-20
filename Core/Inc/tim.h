#ifndef _TIM0_H_
#define _TIM0_H_
#include "gd32f30x_timer.h"
#include "stdint.h"

void TIM0_PWM_Init(uint16_t prescaler,
                   uint32_t period,
                   uint32_t deadtime_ns);
void TIM1_Init(void);
void cal_fmain(float* f, float* Ts, float* PWM_ARR);

static inline void Set_PWM_Compare(float Tcm1,
                                   float Tcm2,
                                   float Tcm3,
                                   float duty)
{
    TIMER_CH0CV(TIMER0)
        = (uint32_t)((float)(TIMER_CAR(TIMER0) + 1) * Tcm1);
    TIMER_CH1CV(TIMER0)
        = (uint32_t)((float)(TIMER_CAR(TIMER0) + 1) * Tcm2);
    TIMER_CH2CV(TIMER0)
        = (uint32_t)((float)(TIMER_CAR(TIMER0) + 1) * Tcm3);
    TIMER_CH3CV(TIMER0)
        = (uint32_t)((float)(TIMER_CAR(TIMER0) + 1) * duty);
}

#endif /* _TIM0_H_ */
