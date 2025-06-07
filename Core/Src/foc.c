
#include "foc.h"

void ClarkeTransform(float_t Ia, float_t Ib, float_t Ic, Clarke_t *out)
{
#if (defined(TWO_PHASE_CURRENT_SENSING))
    out->Ialpha = Ia;
    out->Ibeta = 0.57735026919f * (Ia + 2.0f * Ib); // 0.57735026919f 1/√3
#elif (defined(THREE_PHASE_CURRENT_SENSING))
    out->Ialpha = 0.66666666667f * Ia - 0.33333333333f * (Ib + Ic);
    out->Ibeta = 0.57735026919f * (Ib - Ic);  // 0.57735026919f 1/√3
#endif
}

void ParkTransform(float_t Ialpha, float_t Ibeta, float_t theta, Park_t *out)
{
    float cos_theta = COS(theta);
    float sin_theta = SIN(theta);
    out->Id = Ialpha * cos_theta + Ibeta * sin_theta;
    out->Iq = -Ialpha * sin_theta + Ibeta * cos_theta;
}

void InvParkTransform(float_t Ud, float_t Uq, float_t theta, InvPark_t *out)
{
    float cos_theta = COS(theta);
    float sin_theta = SIN(theta);
    out->Ualpha = Ud * cos_theta - Uq * sin_theta;
    out->Ubeta = Ud * sin_theta + Uq * cos_theta;
}

void Set_PWM_Duty(float_t Ta, float_t Tb, float_t Tc, float_t pwm_arr)
{
#if (defined(GATE_POLARITY_HIGH_ACTIVE))
    TIMER_CH0CV(TIMER0) = (uint32_t)(pwm_arr * Ta);
    TIMER_CH1CV(TIMER0) = (uint32_t)(pwm_arr * Tb);
    TIMER_CH2CV(TIMER0) = (uint32_t)(pwm_arr * Tc);
#elif (defined(GATE_POLARITY_LOW_ACTIVE))
    TIMER_CH0CV(TIMER0) = (uint32_t)(pwm_arr * (1.0f - Ta));
    TIMER_CH1CV(TIMER0) = (uint32_t)(pwm_arr * (1.0f - Tb));
    TIMER_CH2CV(TIMER0) = (uint32_t)(pwm_arr * (1.0f - Tc));
#endif
}

void SVPWM_Generate(float_t Ualpha, float_t Ubeta, float_t Vdc, float_t pwm_arr)
{
    // 计算三个虚拟相电压
    float_t Ua = Ubeta;
    float_t Ub = -0.5f * Ubeta + 0.86602540378f * Ualpha; // cos(60°)=0.5, sin(60°)=√3/2
    float_t Uc = -0.5f * Ubeta - 0.86602540378f * Ualpha;

    // 扇区判断：避免 atan2，改用符号判断
    uint8_t sector = 0;
    if (Ua > 0)
        sector |= 1;
    if (Ub > 0)
        sector |= 2;
    if (Uc > 0)
        sector |= 4;

    // 扇区索引表
    static const uint8_t sector_map[8] = {0, 5, 1, 6, 3, 4, 2, 0}; // 3-bit map -> sector[1~6]
    uint8_t sec = sector_map[sector];

    
    const float Kv = SQRT3 / Vdc;

    // T1/T2计算（使用 Clarke 坐标和扇区矢量合成公式）
    float_t T1 = 0, T2 = 0;
    switch (sec)
    {
    case 1:
        T1 = Kv * (Ualpha - 0.57735026919f * Ubeta); // √3/3 = 0.577
        T2 = Kv * (2.0f * 0.57735026919f * Ubeta);
        break;
    case 2:
        T1 = Kv * (-Ualpha - 0.57735026919f * Ubeta);
        T2 = Kv * (Ualpha - 0.57735026919f * Ubeta);
        break;
    case 3:
        T1 = Kv * (-2.0f * 0.57735026919f * Ubeta);
        T2 = Kv * (Ualpha + 0.57735026919f * Ubeta);
        break;
    case 4:
        T1 = Kv * (-Ualpha + 0.57735026919f * Ubeta);
        T2 = Kv * (-Ualpha - 0.57735026919f * Ubeta);
        break;
    case 5:
        T1 = Kv * (Ualpha - 0.57735026919f * Ubeta);
        T2 = Kv * (-2.0f * 0.57735026919f * Ubeta);
        break;
    case 6:
        T1 = Kv * (Ualpha + 0.57735026919f * Ubeta);
        T2 = Kv * (-Ualpha + 0.57735026919f * Ubeta);
        break;
    default:
        T1 = T2 = 0;
        break;
    }

    // 防止过调制引发的越界
    float_t T_sum = T1 + T2;
    float_t T0 = 1.0f - T_sum;

    if (T0 < 0.0f)
    {
        // 进入过调制，按比例缩放 T1/T2 到 1.0
        float_t scale = 1.0f / T_sum;
        T1 *= scale;
        T2 *= scale;
        T0 = 0.0f;
    }

    float_t T0_half = T0 * 0.5f;

    // 各相占空比 [0.0 - 1.0]
    float_t Ta, Tb, Tc;

    switch (sec)
    {
    case 1:
        Ta = T1 + T2 + T0_half;
        Tb = T2 + T0_half;
        Tc = T0_half;
        break;
    case 2:
        Ta = T1 + T0_half;
        Tb = T1 + T2 + T0_half;
        Tc = T0_half;
        break;
    case 3:
        Ta = T0_half;
        Tb = T1 + T2 + T0_half;
        Tc = T2 + T0_half;
        break;
    case 4:
        Ta = T0_half;
        Tb = T1 + T0_half;
        Tc = T1 + T2 + T0_half;
        break;
    case 5:
        Ta = T2 + T0_half;
        Tb = T0_half;
        Tc = T1 + T2 + T0_half;
        break;
    case 6:
        Ta = T1 + T2 + T0_half;
        Tb = T0_half;
        Tc = T1 + T0_half;
        break;
    default:
        Ta = Tb = Tc = 0.5f;
        break;
    }

    Set_PWM_Duty(Ta, Tb, Tc, pwm_arr);
}

void VF_Control(float freq, float vref, float vdc, float ts, float pwm_arr)
{
    static float theta = 0.0f;

    // 电角度递推：θ += ω·Ts，ω = 2π·f
    theta += 2.0f * M_PI * freq * ts;
    if (theta > 2.0f * M_PI)
        theta -= 2.0f * M_PI;

    // 设置 d轴电压为 0，q轴电压为幅值（绕z轴旋转）
    float Ud = 0.0f;
    float Uq = vref;

    InvPark_t inv;
    InvParkTransform(Ud, Uq, theta, &inv);

    SVPWM_Generate(inv.Ualpha, inv.Ubeta, vdc, pwm_arr);
}