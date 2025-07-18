#ifndef _FOC_TYPES_H_
#define _FOC_TYPES_H_

#include "gd32f30x.h"

/*    Type Definitions   */
typedef enum
{
    INIT,
    IDLE,
    VF_MODE,
    IF_MODE,
    Speed,
    EXIT,
    Identify,
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
    float Position_Offset; // Zero Position
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
    float Theta; /* Electrical angle (rad) */
    float Speed; /* Speed (rpm) */
    float Ud_ref;
    float Uq_ref;
    float Id_ref;
    float Iq_ref;
    float PWM_ARR; /* PWM period */
    float Ts;
    float f;
    FOC_Mode Mode; // 当前控制模式
} FOC_Parameter_t;

typedef struct
{
    float Ialpha;
    float Ibeta;
} Clarke_t;

typedef struct
{
    float Id;
    float Iq;
} Park_t;

typedef struct
{
    float Ualpha;
    float Ubeta;
} InvPark_t;

#endif /* _FOC_TYPES_H_ */
