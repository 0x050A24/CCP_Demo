#ifndef _FOC_H_
#define _FOC_H_

#include "main.h"

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

/*    Type Definitions   */
typedef enum
{
    FOC_MODE_VF,
    FOC_MODE_FOC
} FOC_Mode;

typedef struct
{
    float_t Ia, Ib, Ic; /* Phase currents */
    float_t theta;      /* Electrical angle (rad) */
    float_t Ud, Uq;     /* Voltage components in d-q frame */
    float_t Vdc;        /* DC bus voltage */
    float_t pwm_arr;    /* PWM period */
    FOC_Mode mode;      /* Control mode */
} FOC_Controller_t;

typedef struct
{
    float_t Ialpha;
    float_t Ibeta;
} Clarke_t;

typedef struct
{
    float_t Id;
    float_t Iq;
} Park_t;

typedef struct
{
    float_t Ualpha;
    float_t Ubeta;
} InvPark_t;

/*======================*/
/*    Function Protos   */
/*======================*/

void ClarkeTransform(float_t Ia, float_t Ib, float_t Ic, Clarke_t *out);
void ParkTransform(float_t Ialpha, float_t Ibeta, float_t theta, Park_t *out);
void InvParkTransform(float_t Ud, float_t Uq, float_t theta, InvPark_t *out);
void SVPWM_Generate(float_t Ualpha, float_t Ubeta, float_t Vdc, float_t pwm_arr);
void Set_PWM_Duty(float_t Ta, float_t Tb, float_t Tc, float_t pwm_arr);
void VF_Control(float freq, float vref, float vdc, float ts, float pwm_arr);

#endif /* _FOC_H_ */
