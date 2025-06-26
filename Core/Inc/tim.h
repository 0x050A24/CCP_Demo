#ifndef _TIM0_H_
#define _TIM0_H_

#include "gd32f30x.h"

extern float PWM_ARR;
extern ControlStatus Software_BRK;

void TIM0_PWM_Init(void);

void TIM1_Init(void);

#endif /* _TIM0_H_ */
