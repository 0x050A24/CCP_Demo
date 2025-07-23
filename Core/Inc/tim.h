#ifndef _TIM0_H_
#define _TIM0_H_
#include "gd32f30x_timer.h"
#include "stdint.h"


/* Timer0 configuration */
#define MCU_MAIN_FREQ SystemCoreClock /* 120MHz */
#define MAIN_INT_TIMER_PRESCALER 0
#define MAIN_INT_TIMER_PERIOD 6000 /* 10kHz, 120MHz / 6000 / 2 */
#define MAIN_INT_TIMER_DEADTIME_PERIOD 2000 /* 2us, 120MHz / 6000 / 2 * 2000 = 2us */

#define MAIN_LOOP_FREQ (MCU_MAIN_FREQ / (MAIN_INT_TIMER_PRESCALER + 1) / MAIN_INT_TIMER_PERIOD / 2)
#define MAIN_LOOP_TIME (1.0F / MAIN_LOOP_FREQ) /* 10kHz */

void TIM0_PWM_Init(void);
void TIM1_Init(void);
void cal_fmain(float* f, float* Ts, float* PWM_ARR);

static inline void Set_PWM_Compare(float Tcm1, float Tcm2, float Tcm3)
{
    TIMER_CH0CV(TIMER0) = (uint32_t) ((float) (TIMER_CAR(TIMER0) + 1) * Tcm1);
    TIMER_CH1CV(TIMER0) = (uint32_t) ((float) (TIMER_CAR(TIMER0) + 1) * Tcm2);
    TIMER_CH2CV(TIMER0) = (uint32_t) ((float) (TIMER_CAR(TIMER0) + 1) * Tcm3);
}

#endif /* _TIM0_H_ */
