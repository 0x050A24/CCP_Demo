#ifndef _TIM0_H_
#define _TIM0_H_
#include "stdint.h"
#include "gd32f30x_timer.h"

void TIM0_PWM_Init(void);
void TIM1_Init(void);
void cal_fmain(float *f, float *Ts, float *PWM_ARR);

static inline void Set_PWM_Compare(float Tcm1, float Tcm2, float Tcm3, float pwm_arr)
{
    TIMER_CH0CV(TIMER0) = (uint32_t)(pwm_arr * Tcm1);
    TIMER_CH1CV(TIMER0) = (uint32_t)(pwm_arr * Tcm2);
    TIMER_CH2CV(TIMER0) = (uint32_t)(pwm_arr * Tcm3);
}

#endif /* _TIM0_H_ */
