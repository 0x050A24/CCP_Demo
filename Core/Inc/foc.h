#ifndef _FOC_H_
#define _FOC_H_

#include "main.h"

/*  Gate polarity definition */
#ifndef GATE_POLARITY_HIGH_ACTIVE
#ifndef GATE_POLARITY_LOW_ACTIVE
#define GATE_POLARITY_HIGH_ACTIVE /* Define here */
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


/*    Type Definitions   */
typedef enum
{
    INIT,
    IDLE,
    VF_MODE,
    IF_MODE,
    Speed_Mode,
    EXIT,
    space
} FOC_Mode;

typedef enum
{
    No_Protect = 0,
    Over_Current = 1 << 0,         // 0b0001
    Over_Maximum_Current = 1 << 1, // 0b0010
    Over_Voltage = 1 << 2,         // 0b0100
    Low_Voltage = 1 << 3,          // 0b1000
    Hardware_Fault = 1 << 4,       // 0b10000
    Over_Heat = 1 << 5             // 0b100000
} Protect_Flags;

typedef struct
{
    float Rs;
    float Ld;
    float Lq;
    float Flux;
    float Pn;
    uint16_t Position_Scale;
    float Resolver_Pn;
    float inv_MotorPn;
    float Positon_Offset; // Zero Position
} Motor_Parameter_t;

typedef struct
{
    float Vref_Ud;
    float Vref_Uq;
    float Freq;
    float Theta;
} VF_Parameter_t;

typedef struct
{
    /* data */
    float Kp;             /* Proportional gain */
    float Ki;             /* Integral gain */
    float Kd;             /* Derivative gain */
    float integral;       /* Integral term */
    float previous_error; /* Previous error for derivative calculation */
    float MaxOutput;      /* Maximum output limit */
    float MinOutput;      /* Minimum output limit */
    float output;         /* PID output value */
    float IntegralLimit;  /* Integral limit to prevent windup */
    float Ts;             /* Sample time */
} PID_Controller_t;

typedef struct
{
    float value; // output value
    float slope; // Δvalue/s
    float limit_min;
    float limit_max;
    float target;
    float Ts;
} RampGenerator_t;

typedef struct
{
    float Id_ref;
    float Iq_ref;
    float IF_Freq;
    float Theta;
    ControlStatus Sensor_State;
} IF_Parameter_t;

typedef struct
{
    float_t Theta; /* Electrical angle (rad) */
    float_t Speed; /* Speed (rpm) */
    float_t Ud_ref;
    float_t Uq_ref;
    float_t Id_ref;
    float_t Iq_ref;
    float_t PWM_ARR; /* PWM period */
    FOC_Mode Mode;   // 当前控制模式
} FOC_Parameter_t;

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
extern uint16_t STOP;
extern FOC_Parameter_t FOC_Parameter;

void Gate_state(void);
void FOC_Main(void);
void ClarkeTransform(float_t Ia, float_t Ib, float_t Ic, Clarke_t *out);
void ParkTransform(float_t Ialpha, float_t Ibeta, float_t theta, Park_t *out);
void InvParkTransform(float_t Ud, float_t Uq, float_t theta, InvPark_t *out);
void SVPWM_Generate(float Ualpha, float Ubeta, float inv_Vdc, float pwm_arr);
void Set_PWM_Duty(float_t Ta, float_t Tb, float_t Tc, float_t pwm_arr);
void PID_Controller(float setpoint, float measured_value, PID_Controller_t *PID_Controller);
void Temperature_Protect(void);

#endif /* _FOC_H_ */
