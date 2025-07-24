#ifndef COMMON_MATH_H
#define COMMON_MATH_H

/*  DSP math function    */
#ifndef ARM_DSP
#define ARM_DSP
#endif

#ifdef ARM_DSP
#include "arm_math.h" /* CMSIS-DSP math */  // IWYU pragma: export

#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)

#else
#include <math.h>

#define COS(x) cosf(x)
#define SIN(x) sinf(x)

#endif

/*       Constants      */
#define SQRT3 1.73205080757F
#define SQRT3_2 0.86602540378F        /* √3/2 */
#define M_2PI 6.28318530717958647692F /* 2π */
#define T_2kHz 0.0005F                /* T 2kHz */
#define f_2kHz 2000.0F                /* f 2kHz */
#define T_1kHz 0.0001F                /* T 1kHz */
#define T_200Hz 0.005F                /* T 200Hz */
#define T_10kHz 0.0001F               /* 10kHz sampling time */

#define TIME_2KHZ 0.0005F  /* T 2kHz */
#define FREQ_2KHZ 2000.0F  /* f 2kHz */
#define TIME_1KHZ 0.0001F  /* T 1kHz */
#define TIME_200HZ 0.005F  /* T 200Hz */
#define TIME_10KHZ 0.0001F /* 10kHz sampling time */

/* FOC parameters */
#define SPEED_LOOP_PRESCALER 10.0F /* Speed loop frequency division factor */

#endif /* COMMON_MATH_H */
