#include "leso.h"
#include <stdbool.h>
#include "arm_math.h" /* CMSIS-DSP math */  // IWYU pragma: export
#include "filter.h"
#include "pid.h"
#include "theta_calc.h"
#include "transformation.h"

#define SQRT(x, y) arm_sqrt_f32(x, y)

// Leso_A 和 Leso_B 是为了减少运行时计算时间而预先计算好的系数，用于后续算法中直接使用。

bool  Leso_Enabled          = {0};
static float Leso_Beta1            = {0};
static float Leso_Beta2            = {0};
static float Leso_Rs               = {0};
static float Leso_Ld               = {0};
static float Leso_Lq               = {0};
static float Leso_InvLd            = {0};
static float Leso_InvLq            = {0};
static float Leso_SampleTime       = {0};
static float Leso_SampleFreq_Speed = {0};
static float Leso_Prescaler        = {0};
static float Leso_Pn               = {0};
static float Leso_InvPn            = {0};
static float Leso_Gain             = {0};
static float Leso_Factor           = {0};
static float Leso_Wc               = {0};
static float Leso_Wc_Min           = {0};
static float Leso_Wc_Max           = {0};
static float Leso_We               = {0};
static float Leso_Theta            = {0};
static float Leso_Speed            = {0};
static float Leso_Error            = {0};

static volatile float Leso_Theta_Err = {0};
static volatile float Leso_Speed_Err = {0};
static volatile float Leso_Int_limit = {0};

static Clark_t Leso_Voltage = {0};
static Clark_t Leso_Current = {0};
static Clark_t Leso_CurEst  = {0};
Clark_t Leso_EmfEst  = {0};
Park_t Leso_EmfEst_dq = {0};
Park_t Leso_Emf_Filtered = {0};
Park_t Leso_Emf_Slow_Filtered = {0};
Compensator_t Comp = {0};

static IIR1stFilter_t Leso_Speed_Filter = {0};
static PID_Handler_t  Leso_Theta_PID    = {0};

/**
 * @brief 设置SMO采样时间相关参数
 * @note 该函数应在 Leso_Initialization 之前调用，因为部分参数依赖采样时间进行计算。
 * @param config 指向 SystemTimeConfig_t 结构体的指针，包含采样时间、速度频率和预分频参数
 */
bool Leso_Set_SampleTime(const SystemTimeConfig_t* config)
{
    if (config == NULL)
    {
        return false;
    }

    Leso_SampleTime       = config->current.val;  // 采样时间
    Leso_SampleFreq_Speed = config->speed.inv;    // 速度频率倒数
    Leso_Prescaler        = config->prescaler;    // 预分频系数

    return true;
}

/**
 * @brief 设置SMO参数
 * @note 需要先调用 Leso_Set_SampleTime 函数，再调用本函数进行参数设置。
 * @param param 指向 LESO_Param_t 结构体的指针，包含所需参数
 */
bool Leso_Initialization(const LESO_Param_t* param)
{
    Leso_Gain   = param->wc_gain;
    Leso_Wc_Max = param->wc_max;
    Leso_Wc_Min = param->wc_min;
    Leso_Rs     = param->Rs;
    Leso_Ld     = param->Ld;
    Leso_Lq     = param->Lq;
    Leso_InvLd  = 1.0F / Leso_Ld;
    Leso_InvLq  = 1.0F / Leso_Lq;

    Leso_Int_limit = 1E25F;  // 积分限幅值

    return true;
}

void Leso_Set_Pn(float pole_pairs)
{
    Leso_Pn    = pole_pairs;
    Leso_InvPn = 1.0F / pole_pairs;
}

void Leso_Set_Inductor(Park_t inductance)
{
    Leso_Ld           = inductance.d <= 0.001F ? 0.001F : inductance.d;
    Leso_Lq           = inductance.q <= 0.001F ? 0.001F : inductance.q;
    Leso_InvLd        = 1.0F / Leso_Ld;
    Leso_InvLq        = 1.0F / Leso_Lq;
    // float  leso_Ld    = inductance.d <= 0.001F ? 0.001F : inductance.d;
    // float  leso_Lq    = inductance.q <= 0.001F ? 0.001F : inductance.q;
    // float  leso_InvLd = 1.0F / leso_Ld;
    // float  leso_InvLq = 1.0F / leso_Lq;
    Park_t Idq        = ParkTransform(Leso_Current, Leso_Theta);
    Idq.d             = Idq.d <= 0.5F ? 0.5F : Idq.d;
    // float temp        = (leso_Ld - leso_Lq) / leso_Lq * Idq.d;
    // SQRT(temp, &Leso_Factor);
    // if (Leso_Factor < 1.4F)
    // {
    //     Leso_Factor = 1.4F;
    // }
    // if(Leso_Factor > 1.9F)
    // {
    //     Leso_Factor = 1.9F;
    // }
    Leso_Factor = 1.9F;
}

void Leso_Set_SpeedFilter(float cutoff_freq, float sample_freq)
{
    IIR1stFilter_Init(&Leso_Speed_Filter, cutoff_freq, sample_freq);
}

void Leso_Set_Pid_Handler(PID_Handler_t config)
{
    Leso_Theta_PID = config;
}

void Leso_Set_Voltage(Clark_t voltage)
{
    Leso_Voltage = voltage;
}

void Leso_Set_Current(Clark_t current)
{
    Leso_Current = current;
}

void Leso_Set_Theta(float theta)
{
    Leso_Theta = theta;
}

void Leso_Set_Speed(float speed)
{
    Leso_Speed = speed;
    Leso_We    = rpm2radps(speed) * Leso_Pn;
}

void Leso_Calc_ThetaErr(float ref)
{
    float err      = wrap_theta_2pi(ref - Leso_Theta + PI) - PI;
    Leso_Theta_Err = rad2deg(err);
}

void Leso_Calc_SpeedErr(float ref)
{
    Leso_Speed_Err = ref - Leso_Speed;
}

void Leso_Set_Enabled(bool enabled)
{
    Leso_Enabled = enabled;
}

bool Leso_Get_Enabled(void)
{
    return Leso_Enabled;
}

AngleResult_t Leso_Get_Result(void)
{
    AngleResult_t result = {0};
    result.theta         = Leso_Theta;
    result.speed         = Leso_Speed;
    return result;
}

float Leso_Get_PllErr(void)
{
    return Leso_Error;
}

Clark_t Leso_Get_EmfEst(void)
{
    return Leso_EmfEst;
}

static inline float clamp_f32(float val, float min, float max)
{
    if (val > max)
    {
        return max;
    }
    if (val < min)
    {
        return min;
    }
    return val;
}

void Leso_Update_Beta(void)
{
    Leso_Wc = Leso_Gain * Leso_Factor * Leso_We;
    // if (Leso_Speed < 400.0F) {
    //     Leso_Wc = Leso_Wc_Min;
    // } else {
    if (Leso_Wc > Leso_Wc_Max)
    {
        Leso_Wc = Leso_Wc_Max;
    }
    if (Leso_Wc < Leso_Wc_Min)
    {
        Leso_Wc = Leso_Wc_Min;
    }
    // }

    // 根据带宽计算观测器增益
    Leso_Beta1 = 2.0F * Leso_Wc;
    Leso_Beta2 = Leso_Wc * Leso_Wc;

    Leso_InvLq = 1.0F / Leso_Lq;
}

void Leso_Update_EmfEstA(void)
{
    // 更新电动势估计值
    static float leso_f1a  = 0.0F;
    float        leso_f0   = 0.0F;
    float        leso_b0u  = 0.0F;
    float        leso_err  = 0.0F;
    float        leso_dcur = 0.0F;

    leso_err = Leso_CurEst.a - Leso_Current.a;
    leso_f0  = -Leso_Current.a * Leso_Rs * Leso_InvLq;
    leso_b0u = Leso_Voltage.a * Leso_InvLq;
    leso_f1a -= Leso_Beta2 * Leso_SampleTime * leso_err;

    leso_f1a = clamp_f32(leso_f1a, -Leso_Int_limit, Leso_Int_limit);

    leso_dcur = leso_f0 + leso_b0u + leso_f1a - Leso_Beta1 * leso_err;
    Leso_CurEst.a += leso_dcur * Leso_SampleTime;

    Leso_CurEst.a
        = clamp_f32(Leso_CurEst.a, -Leso_Int_limit, Leso_Int_limit);

    Leso_EmfEst.a = -Leso_Lq * leso_f1a;

    // Buffer_Put(Leso_CurEst.a, 5);
    // Buffer_Put(Leso_Current.a, 6);


}

void Leso_Update_EmfEstB(void)
{
    // 更新电动势估计值
    static float leso_f1b  = 0.0F;
    float        leso_f0   = 0.0F;
    float        leso_b0u  = 0.0F;
    float        leso_err  = 0.0F;
    float        leso_dcur = 0.0F;

    leso_err = Leso_CurEst.b - Leso_Current.b;
    leso_f0  = -Leso_Current.b * Leso_Rs * Leso_InvLq;
    leso_b0u = Leso_Voltage.b * Leso_InvLq;
    leso_f1b -= Leso_Beta2 * Leso_SampleTime * leso_err;

    leso_f1b = clamp_f32(leso_f1b, -Leso_Int_limit, Leso_Int_limit);

    leso_dcur = leso_f0 + leso_b0u + leso_f1b - Leso_Beta1 * leso_err;
    Leso_CurEst.b += leso_dcur * Leso_SampleTime;

    Leso_CurEst.b
        = clamp_f32(Leso_CurEst.b, -Leso_Int_limit, Leso_Int_limit);

    Leso_EmfEst.b = -Leso_Lq * leso_f1b;

    // Buffer_Put(Leso_CurEst.b, 7);
    // Buffer_Put(Leso_Current.b, 8);
}

// static inline float compensate_theta(float theta, float omega) {
//     // 角度补偿
//     float comp = 0.0F;
//     ATAN2(omega, Leso_Gain, &comp);
//     return theta + comp;
// }

// static inline float pll_update(float error, bool reset) {
//     // 更新锁相环
//     float omega = Pid_Update(error, reset, &Leso_Theta_PID);
//     Leso_Theta += omega * Leso_SampleTime;
//     if (Leso_Theta > M_2PI) {
//         Leso_Theta -= M_2PI;
//     }
//     if (Leso_Theta < 0.0F) {
//         Leso_Theta += M_2PI;
//     }
//     Leso_Theta = wrap_theta_2pi(Leso_Theta);
//     return omega;
// }

static inline float calculate_error(Clark_t emf, float angle)
{
    float angleErr   = 0.0F;
    float errorAlpha = 0.0F;
    float errorBeta  = 0.0F;

    if (!Leso_Enabled)
    {
        return angleErr;
    }

    float sin_leso = SIN(angle);
    float cos_leso = COS(angle);

    errorAlpha = -emf.a * cos_leso;
    errorBeta  = emf.b * sin_leso;
    angleErr   = errorAlpha - errorBeta;

    float norm = 0.0F;
    SQRT(emf.a * emf.a + emf.b * emf.b, &norm);
    if (norm > 0.0001F)
    {
        angleErr /= norm;
    }
    else
    {
        angleErr = 0.0F;
    }

    return angleErr;
}

// static inline float calculate_speed(float omega) {
//     static uint16_t leso_count = 0x0000U;
//     static float    leso_integ = 0.0F;
//     float           speed      = 0.0F;
//     leso_integ += radps2rpm(omega) * Leso_InvPn * 0.1F;
//     leso_count++;
//     if (leso_count < 0x000AU) {
//         return Leso_Speed;
//     }
//     leso_count = 0x0000U;
//     speed      = IIR1stFilter_Update(&Leso_Speed_Filter, leso_integ);
//     leso_integ = 0.0F;
//     return speed;
// }

void Leso_Update(void)
{
    Leso_Error = calculate_error(Leso_EmfEst, Leso_Theta);
    // 计算电动势的相位角
    // float omega = pll_update(Leso_Error, !Leso_Enabled);
    // Leso_Speed  = calculate_speed(omega);
}
