#ifndef _FOC_H_
#define _FOC_H_

#include "main.h"
#include "foc_types.h"
#include "injection.h"

/*  Gate polarity definition */
#ifndef GATE_POLARITY_HIGH_ACTIVE
#ifndef GATE_POLARITY_LOW_ACTIVE
#define GATE_POLARITY_LOW_ACTIVE /* Define here */
#endif
#endif

#if !defined(GATE_POLARITY_HIGH_ACTIVE) && !defined(GATE_POLARITY_LOW_ACTIVE)
#error "Please define GATE_POLARITY_HIGH_ACTIVE or GATE_POLARITY_LOW_ACTIVE in foc.h"
#endif

/* Current sensing phase setting */
#ifndef TWO_PHASE_CURRENT_SENSING
#ifndef THREE_PHASE_CURRENT_SENSING
#define THREE_PHASE_CURRENT_SENSING /* Define here */
#endif
#endif

#if !defined(TWO_PHASE_CURRENT_SENSING) && !defined(THREE_PHASE_CURRENT_SENSING)
#error "Please define TWO_PHASE_CURRENT_SENSING or THREE_PHASE_CURRENT_SENSING in foc.h"
#endif

/*  DSP math function    */
#ifndef ARM_DSP
#define ARM_DSP
#endif

#ifdef ARM_DSP
#include "arm_math.h" /* CMSIS-DSP math */

#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)

#else
#include <math.h>

#define COS(x) cosf(x)
#define SIN(x) sinf(x)

#endif

/*       Constants      */
#define SQRT3 1.73205080757f
#define SQRT3_2 0.86602540378f /* √3/2 */
#define T_Main 0.0005f         /* T 2kHz */
#define T_2kHz 0.0005f         /* T 2kHz */
#define f_2kHz 2000.0f         /* f 2kHz */
#define T_1kHz 0.0001f         /* T 1kHz */
#define T_200Hz 0.005f         /* T 200Hz */
#define T_10kHz 0.0001f        /* 10kHz sampling time */




/*======================*/
/*    Function Protos   */
/*======================*/
extern uint16_t STOP;
extern FOC_Parameter_t FOC_Parameter;

void Gate_state(void);
void FOC_Main(void);

void Set_PWM_Duty(float_t Ta, float_t Tb, float_t Tc, float_t pwm_arr);
void PID_Controller(float setpoint, float measured_value, PID_Controller_t *PID_Controller);
void Temperature_Protect(void);

#endif /* _FOC_H_ */
