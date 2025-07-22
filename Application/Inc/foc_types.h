#ifndef _FOC_TYPES_H_
#define _FOC_TYPES_H_

#include "stdint.h"

typedef enum {Disable = 0, Enable = !Disable} EnableStatus, CommandStatus;

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
    float Position_Offset;  // Zero Position
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
    float value;  // output value
    float slope;  // Δvalue/s
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
    EnableStatus Sensor_State;
} IF_Parameter_t;

typedef struct
{
    float Ia;
    float Ib;
    float Ic;
    float Id;
    float Iq;
    float I_Max;
    float Udc;
    float inv_Udc;
    uint16_t Position;  // position sensor usually up to 16bits resolution
    float Theta;        /* Electrical angle (rad) */
    float Speed;        /* Speed (rpm) */
    float Ud_ref;
    float Uq_ref;
    float Id_ref;
    float Iq_ref;
    float PWM_ARR; /* PWM period */
    float Tcm1;
    float Tcm2;
    float Tcm3;
    float Ts;
    float f;
    FOC_Mode Mode;  // 当前控制模式
} FOC_Parameter_t;

typedef struct
{
    float Ialpha;
    float Ibeta;
} Clarke_t;

// struct Park_t is commented out as struct FOC_Parameter_t takes care of Id and Iq
// typedef struct
// {
//     float Id;
//     float Iq;
// } Park_t;

typedef struct
{
    float Ualpha;
    float Ubeta;
} InvPark_t;

typedef struct
{
    float Tcm1;
    float Tcm2;
    float Tcm3;
} SVPWM_t;

#endif /* _FOC_TYPES_H_ */
