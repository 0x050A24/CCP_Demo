/**
 * @file sensorless_interface.c
 * @brief 无传感器控制接口层实现
 * @author FRECON
 * @date 2025年7月28日
 * @version 2.0
 */

#include "sensorless_interface.h"
#include "parameters.h"

#include <stdbool.h>
#include "buffer.h"
#include "filter.h"
#include "flying.h"
#include "hf_injection.h"
#include "leso.h"
#include "motor.h"
#include "reciprocal.h"
#include "transformation.h"
#include <math.h>

static volatile bool Sensorless_Enabled = {0};

static bool  Sensorless_Reset          = {0};
static bool  Sensorless_Reset_Prev     = {0};
static float Sensorless_Threshold_Hfi  = {0};
static float Sensorless_Threshold_Leso = {0};
static float Sensorless_Switch_Speed   = {0};
static float Sensorless_SpeedRef       = {0};
static float Sensorless_SpeedTarget    = {0};
static float Sensorless_SpeedFdbk      = {0};
static float Sensorless_SpeedEst       = {0};
static float Sensorless_ThetaEst       = {0};
static float Sensorless_SampleTime     = {0};
static float Sensorless_InvPn          = {0};
static float Sensorless_ThetaErr       = {0};
static float Sensorless_SpeedErr       = {0};

volatile float Sensorless_ThetAdj = {0};

static PID_Handler_t  Sensorless_Theta_PID    = {0};
static IIR1stFilter_t Sensorless_SpeedFilter1 = {0};
static IIR2ndFilter_t Sensorless_SpeedFilter2 = {0};
float Sensorless_PLL_Wc = 0.0F;

static sensorless_method_t Sensorless_Method = FLYING;

bool restore_states(void)
{
    Sensorless_Theta_PID.integral       = 0.0F;
    Sensorless_Theta_PID.previous_error = 0.0F;
    Sensorless_Theta_PID.output         = 0.0F;
    Sensorless_ThetaEst                 = 0.0F;
    Sensorless_SpeedEst                 = 0.0F;

    return true;
}

bool Sensorless_Set_SampleTime(const SystemTimeConfig_t* config)
{
    if (config == NULL)
    {
        return false;
    }

    Sensorless_SampleTime = config->current.val;  // 采样时间

    return true;
}

bool Sensorless_Initialization(const Sensorless_Param_t* param)
{
    if (param == NULL)
    {
        return false;
    }

    Sensorless_Threshold_Hfi  = param->switch_speed + param->hysteresis;
    Sensorless_Threshold_Leso = param->switch_speed - param->hysteresis;
    Sensorless_Switch_Speed   = param->switch_speed;

    return true;
}

bool Sensorless_Set_SpeedFilter(float cutoff_freq, float sample_freq)
{
    if (cutoff_freq <= 0.0F || sample_freq <= 0.0F)
    {
        return false;
    }
    IIR2ndFilter_Init(
        &Sensorless_SpeedFilter2, cutoff_freq, sample_freq);
    IIR1stFilter_Init(
        &Sensorless_SpeedFilter1, cutoff_freq, sample_freq);
    return true;
}

bool Sensorless_Set_PidParams(const PID_Handler_t* pid_handler)
{
    if (pid_handler == NULL)
    {
        return false;
    }
    Sensorless_Theta_PID = *pid_handler;
    return true;
}

bool Sensorless_Set_MotorParams(const MotorParam_t* motor_param)
{
    if (motor_param == NULL || motor_param->inv_MotorPn <= 0)
    {
        return false;
    }

    Sensorless_InvPn = motor_param->inv_MotorPn;
    return true;
}

bool Sensorless_Set_ResetFlag(bool reset)
{
    Sensorless_Reset = reset;
    return Sensorless_Reset;
}

bool Sensorless_Get_Reset(void)
{
    return Sensorless_Reset;
}

bool Sensorless_Set_Method(sensorless_method_t method, bool enable)
{
    if (enable)
    {
        Sensorless_Method |= method;
    }
    else
    {
        Sensorless_Method &= ~method;
    }

    return true;
}

sensorless_method_t Sensorless_Get_Method(void)
{
    return Sensorless_Method;
}

Clark_t Sensorless_Get_SmoEmf(void)
{
    return Leso_Get_EmfEst();
}

bool Sensorless_Set_Voltage(Clark_t voltage)
{
    if (Hfi_Get_Enabled())
    {
    }

    if (Leso_Get_Enabled())
    {
        Leso_Set_Voltage(voltage);
    }

    return true;
}

bool Sensorless_Set_Current(Clark_t current)
{
    // if (Hfi_Get_Enabled()) {
    //     Hfi_Set_Current(current);
    // }

    Leso_Set_Current(current);

    return true;
}

void Sensorless_Set_SpeedFdbk(float fdbk)
{
    Sensorless_SpeedFdbk = fdbk;
}

void Sensorless_Set_SpeedRef(float ref)
{
    Sensorless_SpeedRef = ref;
}

void Sensorless_Set_SpeedTarget(float ref)
{
    Sensorless_SpeedTarget = ref;
}

// void Sensorless_Set_Angle(float angle) {
//     // Sensorless_ThetaEst = angle;
// }

AngleResult_t Sensorless_Get_Error(void)
{
    return (AngleResult_t){.theta = Sensorless_ThetaErr,
                           .speed = Sensorless_SpeedErr};
}

bool Sensorless_Calculate_Err(AngleResult_t result)
{
    if (!Sensorless_Enabled)
    {
        return false;
    }

    float theta = result.theta;
    float speed = result.speed;
    float error = 0.0F;

    error = wrap_theta_2pi(theta - Sensorless_ThetaEst + PI) - PI;
    Sensorless_ThetaErr = rad2deg(error);
    Sensorless_SpeedErr = speed - Sensorless_SpeedEst;

    Hfi_Calc_ThetaErr(theta);
    Hfi_Calc_SpeedErr(speed);

    Leso_Calc_ThetaErr(theta);
    Leso_Calc_SpeedErr(speed);

    return true;
}

static inline float pll_update(float error, bool reset)
{
    // 更新锁相环
    float omega = Pid_Update(error, reset, &Sensorless_Theta_PID);

    if (reset)
    {
        return omega;
    }

    Sensorless_ThetaEst += omega * Sensorless_SampleTime;
    if (Sensorless_ThetaEst > M_2PI)
    {
        Sensorless_ThetaEst -= M_2PI;
    }
    if (Sensorless_ThetaEst < 0.0F)
    {
        Sensorless_ThetaEst += M_2PI;
    }
    Sensorless_ThetaEst = wrap_theta_2pi(Sensorless_ThetaEst);
    return omega;
}

static inline float calculate_speed(float omega)
{
    static uint16_t speed_cnt = 0x0000U;
    static float    speed_int = 0.0F;
    // float           speed1    = 0.0F;
    // float           speed2    = 0.0F;
    float speed = 0.0F;
    speed_int += radps2rpm(omega) * Sensorless_InvPn * 0.1F;
    speed_cnt++;
    if (speed_cnt < 0x000AU)
    {
        return Sensorless_SpeedEst;
    }
    speed_cnt = 0x0000U;
    // speed1 = IIR1stFilter_Update(&Sensorless_SpeedFilter1, speed_int);
    // speed2 = IIR2ndFilter_Update(&Sensorless_SpeedFilter2, speed_int);
    speed = IIR2ndFilter_Update(&Sensorless_SpeedFilter2, speed_int);
    // if (Sensorless_Method == LES_OBSERVER) {
    //     speed = speed2;
    // } else {
    //     speed = speed2;
    // }
    speed_int           = 0.0F;
    Sensorless_SpeedEst = speed;
    return speed;
}

AngleResult_t Sensorless_Update_Position(void)
{
    if (!Sensorless_Enabled)
    {
        return (AngleResult_t){
            .speed = Sensorless_SpeedEst,
            .theta = Sensorless_ThetaEst + Sensorless_ThetAdj};
    }
    float error = 0.0F;
    float omega = 0.0F;
    float speed = 0.0F;
    if (fabsf(Sensorless_SpeedTarget)
        >= Sensorless_Switch_Speed + 10.0F)
    {
        if (fabsf(Sensorless_SpeedRef) >= Sensorless_Switch_Speed)
        {
            if (fabsf(Sensorless_SpeedFdbk) >= Sensorless_Switch_Speed)
            {
                Sensorless_Method = LES_OBSERVER;
                error             = Leso_Get_PllErr();
            }
            else
            {
                Sensorless_Method = HF_INJECTION;
                error             = Hfi_Get_PllErr();
            }
        }
        else
        {
            if (fabsf(Sensorless_SpeedFdbk) <= Sensorless_Switch_Speed)
            {
                Sensorless_Method = HF_INJECTION;
                error             = Hfi_Get_PllErr();
            }
            else
            {
                Sensorless_Method = LES_OBSERVER;
                error             = Leso_Get_PllErr();
            }
        }
    }
    else if (fabsf(Sensorless_SpeedTarget)
             <= Sensorless_Switch_Speed - 10.0F)
    {
        if (fabsf(Sensorless_SpeedRef) <= Sensorless_Switch_Speed)
        {
            if (fabsf(Sensorless_SpeedFdbk) <= Sensorless_Switch_Speed)
            {
                Sensorless_Method = HF_INJECTION;
                error             = Hfi_Get_PllErr();
            }
            else
            {
                Sensorless_Method = LES_OBSERVER;
                error             = Leso_Get_PllErr();
            }
        }
        else
        {
            if (fabsf(Sensorless_SpeedFdbk) >= Sensorless_Switch_Speed)
            {
                Sensorless_Method = LES_OBSERVER;
                error             = Leso_Get_PllErr();
            }
            else
            {
                Sensorless_Method = HF_INJECTION;
                error             = Hfi_Get_PllErr();
            }
        }
    }
    else
    {
        if (Sensorless_Method == HF_INJECTION)
        {
            error = Hfi_Get_PllErr();
        }
        else
        {
            error = Leso_Get_PllErr();
        }
    }
    omega = pll_update(error, Sensorless_Reset);
    speed = calculate_speed(omega);
    //Buffer_Put(error, 9);

    Leso_Set_Theta(Sensorless_ThetaEst);
    Leso_Set_Speed(speed);

    static volatile float PLLKPMAX = 80.0F;
    static volatile float PLLKIMAX = 1600.0F;

    if (speed > 500.0F)
    {
        Sensorless_PLL_Wc = SENSORLESS_PLL_WC + (speed - 500.0F) * 0.075F;
        Sensorless_Theta_PID.Ki = Sensorless_PLL_Wc * Sensorless_PLL_Wc;
        Sensorless_Theta_PID.Kp = 2.0F * Sensorless_PLL_Wc;
        if (Sensorless_Theta_PID.Ki > PLLKIMAX)
        {
            Sensorless_Theta_PID.Ki = PLLKIMAX;
        }
        if (Sensorless_Theta_PID.Kp > PLLKPMAX)
        {
            Sensorless_Theta_PID.Kp = PLLKPMAX;
        }
    }
    else
    {
        Sensorless_Theta_PID.Ki = 625.0F;
        Sensorless_Theta_PID.Kp = 50.0F;
    }
    Hfi_Set_Theta(Sensorless_ThetaEst);

    return (AngleResult_t){
        .speed = Sensorless_SpeedEst,
        .theta = Sensorless_ThetaEst + Sensorless_ThetAdj};
}

static inline void enable_smo(bool enable)
{
    if (enable)
    {
        if (!Leso_Get_Enabled())
        {
            Leso_Set_Enabled(true);
        }
    }
    else
    {
        if (Leso_Get_Enabled())
        {
            Leso_Set_Enabled(false);
        }
    }
}

static inline void enable_hfi(bool enable)
{
    if (enable)
    {
        if (!Hfi_Get_Enabled())
        {
            Hfi_Set_Enabled(true);
            float estimate = 0.0F, target = 0.0F, error = 0.0F;
            estimate = Hfi_Get_Result().theta;
            target   = Sensorless_ThetaEst;
            error    = wrap_theta_2pi(target - estimate + PI) - PI;
            if (error >= M_PI_4 || error <= -M_PI_4)
            {
                Hfi_Set_InitialPosition(target);
            }
        }
    }
    else
    {
        if (Hfi_Get_Enabled())
        {
            Hfi_Set_Enabled(false);
        }
    }
}

static inline void judge_strategy(float ref, float fdbk)
{
    if (fabsf(ref) >= Sensorless_Threshold_Leso)
    {
        if (fabsf(fdbk) >= Sensorless_Threshold_Leso)
        {
            enable_smo(true);
        }
    }
    else
    {
        if (fabsf(fdbk) < Sensorless_Threshold_Leso)
        {
            enable_smo(false);
        }
    }
    if (fabsf(ref) <= Sensorless_Threshold_Hfi)
    {
        if (fabsf(fdbk) <= Sensorless_Threshold_Hfi)
        {
            enable_hfi(true);
        }
    }
    else
    {
        if (fabsf(fdbk) > Sensorless_Threshold_Hfi)
        {
            enable_hfi(false);
        }
    }
}

bool Sensorless_Calculate(void)
{
    if (Sensorless_Reset)
    {
        if (!Sensorless_Reset_Prev)
        {
            // restore_states();
        }
        return false;
    }
    else if (Sensorless_Reset_Prev)
    {
        Sensorless_Set_Method(FLYING, true);
        Flying_Set_Enabled(true);
    }

    if (Flying_Is_Completed())
    {
        Sensorless_Set_Method(FLYING, false);
    }
    else
    {
        Flying_Update(Sensorless_Reset);
    }

    judge_strategy(Sensorless_SpeedRef, Sensorless_SpeedFdbk);

    if (Hfi_Get_Enabled())
    {
        Hfi_Update();
    }

    if (Leso_Get_Enabled())
    {
        Leso_Update_Beta();
        Leso_Update_EmfEstA();
        Leso_Update_EmfEstB();
        Leso_Update();
    }

    Sensorless_Reset_Prev = Sensorless_Reset;

    return true;
}

Park_t Sensorless_Inject_Voltage(Park_t voltage)
{
    if (!Sensorless_Enabled)
    {
        return voltage;
    }

    if (Sensorless_Reset)
    {
        return voltage;
    }

    if (Hfi_Get_Enabled())
    {
        Park_t inj = Hfi_Get_Inject_Voltage();
        voltage.d += inj.d;
        voltage.q += inj.q;
        return voltage;
    }

    return voltage;
}

Clark_t Sensorless_FilterCurrent(Clark_t current)
{
    if (!Sensorless_Enabled)
    {
        return current;
    }

    if (Sensorless_Reset)
    {
        return current;
    }

    if (!Hfi_Get_Enabled())
    {
        return current;
    }

    return Hfi_Process_Current(current);
}
