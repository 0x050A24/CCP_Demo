#ifndef _FOC_H_
#define _FOC_H_

#include "foc_types.h"

/*  Gate polarity definition */
#ifndef GATE_POLARITY_HIGH_ACTIVE
#ifndef GATE_POLARITY_LOW_ACTIVE
#define GATE_POLARITY_LOW_ACTIVE /* Define here */
#endif
#endif

#if !defined(GATE_POLARITY_HIGH_ACTIVE) && !defined(GATE_POLARITY_LOW_ACTIVE)
#error                                                                         \
    "Please define GATE_POLARITY_HIGH_ACTIVE or GATE_POLARITY_LOW_ACTIVE in foc.h"
#endif

/* Current sensing phase setting */
#ifndef TWO_PHASE_CURRENT_SENSING
#ifndef THREE_PHASE_CURRENT_SENSING
#define THREE_PHASE_CURRENT_SENSING /* Define here */
#endif
#endif

#if !defined(TWO_PHASE_CURRENT_SENSING) && !defined(THREE_PHASE_CURRENT_SENSING)
#error                                                                         \
    "Please define TWO_PHASE_CURRENT_SENSING or THREE_PHASE_CURRENT_SENSING in foc.h"
#endif

/*  DSP math function    */
#ifndef ARM_DSP
#define ARM_DSP
#endif

#ifdef ARM_DSP
#include "arm_math.h" /* CMSIS-DSP math */ // IWYU pragma: export

#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)

#else
#include <math.h>

#define COS(x) cosf(x)
#define SIN(x) sinf(x)

#endif

/*       Constants      */
#define SQRT3 1.73205080757F
#define SQRT3_2 0.86602540378F /* √3/2 */
#define M_2PI 6.28318530717958647692F /* 2π */
#define T_2kHz 0.0005F         /* T 2kHz */
#define f_2kHz 2000.0F         /* f 2kHz */
#define T_1kHz 0.0001F         /* T 1kHz */
#define T_200Hz 0.005F         /* T 200Hz */
#define T_10kHz 0.0001F        /* 10kHz sampling time */


// Since CCP demanded struct FOC is Global Variable, make it visible to main ISR //
extern SVPWM_t SVPWM;
extern FOC_Parameter_t FOC;


void FOC_Main(void);

void FOC_UpdateMainFrequency(float f, float Ts, float PWM_ARR);

static inline void FOC_UpdateCurrent(float Ia, float Ib, float Ic)
{
    FOC.Ia = Ia;
    FOC.Ib = Ib;
    FOC.Ic = Ic;
}

static inline void FOC_UpdateVoltage(float Udc, float inv_Udc)
{
    FOC.Udc = Udc;
    FOC.inv_Udc = inv_Udc;
}

static inline void FOC_UpdatePosition(uint16_t Position)
{
    FOC.Position = Position;
}

#endif /* _FOC_H_ */
