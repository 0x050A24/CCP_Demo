
#include "foc.h"

uint16_t STOP = 1;

volatile Protect_Flags Protect_Flag;
Motor_Parameter_t Motor;
FOC_Parameter_t FOC;
VF_Parameter_t VF;
IF_Parameter_t IF;
PID_Controller_t Id_PID;
PID_Controller_t Iq_PID;
PID_Controller_t Speed_PID;
InvPark_t Inv_Park;
Clarke_t Clarke;
Park_t Park;

float I_Max = 10.0f;
float theta_mech = 0.0f;
float theta_elec = 0.0f;
float theta_factor = 0.0f; // Resolver to mechanic angle conversion factor

float Speed_Ref = 0.0f;

static inline void Current_Protect(void);
static inline float Get_Theta(float Freq, float Theta);
static inline void Parameter_Init(void);
static inline void Theta_Process(void);
static inline float wrap_theta_2pi(float theta);

extern uint16_t receive;

void FOC_Main(void)
{
    ADC_Read_Injection();
    Current_Protect();
    Gate_state();
    ReadPosition();
    Theta_Process();

    ClarkeTransform(Ia, Ib, Ic, &Clarke);

    switch (FOC.Mode)
    {
    case INIT:
    {
        Parameter_Init();
        if (Udc > 200.0f) // 200V
        {
            // timer_interrupt_enable(TIMER0, TIMER_INT_BRK); // 启用BRK中断
            ADC_Calibration();
        }
        FOC.Mode = IDLE;
        break;
    }
    case IDLE:
    {
        STOP = 1;
        break;
    }
    case VF_MODE:
    {
        VF.Theta = Get_Theta(VF.Freq, VF.Theta);
        FOC.Theta = VF.Theta;
        FOC.Ud_ref = VF.Vref_Ud;
        FOC.Uq_ref = VF.Vref_Uq;
        break;
    }
    case IF_MODE:
    {
        if (IF.Resolver_State == ENABLE)
        {
            IF.Theta = FOC.Theta;
        }
        else
        {
            IF.Theta = Get_Theta(IF.IF_Freq, IF.Theta);
        }
        FOC.Theta = IF.Theta;

        ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &Park);

        PID_Controller(IF.Id_ref, Park.Id, &Id_PID);
        PID_Controller(IF.Iq_ref, Park.Iq, &Iq_PID);

        FOC.Ud_ref = Id_PID.output;
        FOC.Uq_ref = Iq_PID.output;

        break;
    }
    case Speed_Mode:
    {

        ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &Park);
        static uint16_t Speed_Count = 0;
        Speed_Count++;
        if (Speed_Count > 9)
        {
            Speed_Count = 0;
            PID_Controller(Speed_Ref, FOC.Speed, &Speed_PID);
        }

        PID_Controller(0.0f, Park.Id, &Id_PID);
        PID_Controller(Speed_PID.output, Park.Iq, &Iq_PID);

        FOC.Ud_ref = Id_PID.output;
        FOC.Uq_ref = Iq_PID.output;

        break;
    }
    case EXIT:
    {
        STOP = 1;
        timer_interrupt_disable(TIMER0, TIMER_INT_BRK); // 禁用BRK中断
        FOC.Ud_ref = 0.0f;
        FOC.Uq_ref = 0.0f;

        break;
    }
    default:
    {
        STOP = 1;
        FOC.Mode = IDLE;
        break;
    }
    }

    InvParkTransform(FOC.Ud_ref, FOC.Uq_ref, FOC.Theta, &Inv_Park);
    SVPWM_Generate(Inv_Park.Ualpha, Inv_Park.Ubeta, inv_Udc, FOC.PWM_ARR);
}

void Gate_state(void)
{

    if (Protect_Flag != No_Protect)
    {
        STOP = 1;
    }
    if (STOP)
    {
        // 软件触发 BRK
        Software_BRK = ENABLE;
        TIMER_SWEVG(TIMER0) |= TIMER_SWEVG_BRKG;
    }
    else
    {
        // STOP = 0，尝试恢复
        // if (gpio_input_bit_get(GPIOE, GPIO_PIN_15) == SET)
        {
            Software_BRK = DISABLE;
            timer_primary_output_config(TIMER0, ENABLE); // 恢复 MOE
            STOP = 0;
        }
        // else
        //{
        //     STOP = 1;
        // }
    }
}

void Current_Protect(void)
{
    if ((Ia > 0.9 * I_Max || Ia < -0.9 * I_Max) ||
        (Ib > 0.9 * I_Max || Ib < -0.9 * I_Max) ||
        (Ic > 0.9 * I_Max || Ic < -0.9 * I_Max))
    {
        uint16_t Current_Count = 0;
        Current_Count++;
        if (Current_Count > 10)
        {
            STOP = 1;
            Protect_Flag |= Over_Current;
            Current_Count = 0;
        }
    }
    if ((Ia > I_Max || Ia < -1 * I_Max) ||
        (Ib > I_Max || Ib < -1 * I_Max) ||
        (Ic > I_Max || Ic < -1 * I_Max))
    {
        STOP = 1;
        Protect_Flag |= Over_Maximum_Current;
    }
}

void Temperature_Protect(void)
{
    if (Temperature > 35.0f)
    {
        gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
    }
    if (Temperature > 90.0f)
    {
        STOP = 1;
        Protect_Flag |= Over_Heat;
    }
}

void Parameter_Init(void)
{
    memset(&VF, 0, sizeof(VF_Parameter_t));
    memset(&FOC, 0, sizeof(FOC_Parameter_t));
    memset(&Id_PID, 0, sizeof(PID_Controller_t));
    memset(&Iq_PID, 0, sizeof(PID_Controller_t));
    memset(&Speed_PID, 0, sizeof(PID_Controller_t));
    memset(&Inv_Park, 0, sizeof(InvPark_t));
    memset(&Motor, 0, sizeof(Motor_Parameter_t));

    STOP = 1;
    Protect_Flag = No_Protect;

    Motor.Pn = 5;
    Motor.Resolver_Pn = 1;
    Motor.Resolver_Scale = 65535;
    Motor.inv_MotorPn = 1.0f / Motor.Pn; // 1/Pn
    Motor.Rs = 1.25f;
    Motor.Ld = 0.006f; // 6mH
    Motor.Lq = 0.009f; // 9mH
    Motor.Positon_Offset = 6396.0f;
    theta_factor = 2.0f * M_PI / ((Motor.Resolver_Scale + 1) * Motor.Resolver_Pn);

    Speed_PID.Kp = 0.0f;
    Speed_PID.Ki = 0.0f;
    Speed_PID.Kd = 0.0f;
    Speed_PID.MaxOutput = 0.7 * I_Max; // Maximum Iq
    Speed_PID.MinOutput = -0.7 * I_Max;
    Speed_PID.IntegralLimit = 0.7 * I_Max;
    Speed_PID.previous_error = 0.0f;
    Speed_PID.integral = 0.0f;
    Speed_PID.output = 0.0f;
    Speed_PID.Ts = T_2kHz;

    Id_PID.Kp = 7.5398223686f;
    Id_PID.Ki = 1570.796326794f;
    Id_PID.Kd = 0.0f;
    Id_PID.MaxOutput = 50.0f; // Maximum Udc/sqrt(3)
    Id_PID.MinOutput = -50.0f;
    Id_PID.IntegralLimit = 50.0f;
    Id_PID.previous_error = 0.0f;
    Id_PID.integral = 0.0f;
    Id_PID.output = 0.0f;
    Id_PID.Ts = T_2kHz;

    Iq_PID.Kp = 11.30973355f;
    Iq_PID.Ki = 1570.796326794f;
    Iq_PID.Kd = 0.0f;
    Iq_PID.MaxOutput = 50.0f;
    Iq_PID.MinOutput = -50.0f;
    Iq_PID.IntegralLimit = 50.0f;
    Iq_PID.previous_error = 0.0f;
    Iq_PID.integral = 0.0f;
    Iq_PID.output = 0.0f;
    Iq_PID.Ts = T_2kHz;

    VF.Vref_Ud = 0.0f;
    VF.Vref_Uq = 0.0f;
    VF.Freq = 0.0f;
    VF.Theta = 0.0f;

    IF.Id_ref = 0.0f;
    IF.Iq_ref = 0.0f;
    IF.IF_Freq = 0.0f;
    IF.Theta = 0.0f;
    IF.Resolver_State = DISABLE;

    FOC.PWM_ARR = PWM_ARR;
}

void PID_Controller(float setpoint, float measured_value, PID_Controller_t *PID_Controller)
{
    float difference = setpoint - measured_value;
    float integral = PID_Controller->integral;
    float derivative = difference - PID_Controller->previous_error;

    if (STOP == 1)
    {
        difference = 0.0f;
        integral = 0.0f;
        derivative = 0.0f;
    }
    // Proportional term
    float P = PID_Controller->Kp * difference;
    // Integral term
    integral += PID_Controller->Ki * difference * PID_Controller->Ts;
    // Derivative term
    float D = PID_Controller->Kd * derivative;
    // Calculate output
    float output_value = P + integral + D;
    // Clamp output to limits
    if (output_value > PID_Controller->MaxOutput)
    {
        output_value = PID_Controller->MaxOutput;
    }
    else if (output_value < PID_Controller->MinOutput)
    {
        output_value = PID_Controller->MinOutput;
    }
    if (integral > PID_Controller->IntegralLimit)
        integral = PID_Controller->IntegralLimit;
    else if (integral < -PID_Controller->IntegralLimit)
        integral = -PID_Controller->IntegralLimit;
    // Update integral and previous error for next iteration
    PID_Controller->integral = integral;
    PID_Controller->previous_error = difference;
    // Return the output value
    PID_Controller->output = output_value;
}

static inline float wrap_theta_2pi(float theta)
{
    if (theta >= 2.0f * M_PI)
        theta -= 2.0f * M_PI;
    else if (theta < 0.0f)
        theta += 2.0f * M_PI;
    return theta;
}

typedef struct
{
    float a;      // 反馈系数（= 极点位置）
    float y_last; // 上一次输出值
} LowPassFilter_t;

LowPassFilter_t Speed_Filter = {
    .a = 0.9685841f,
    .y_last = 0.0f};

static inline float LowPassFilter_Update(LowPassFilter_t *filter, float x)
{
    float y = filter->a * filter->y_last + (1.0f - filter->a) * x;
    filter->y_last = y;
    return y;
}

static inline void Theta_Process(void)
{
    //< use multiple instead of divide >//
    theta_mech = (Position_Data - Motor.Positon_Offset) * theta_factor;
    theta_elec = theta_mech * Motor.Pn;

    FOC.Theta = wrap_theta_2pi(theta_elec);
    static float last_theta = 0.0f;
    float delta_theta = theta_mech - last_theta;

    // Wrap-around correction（处理 2π -> 0 或 0 -> 2π 的跳变）
    if (delta_theta > M_PI) // 超过半圈，说明实际是逆转
        delta_theta -= 2.0f * M_PI;
    else if (delta_theta < -M_PI) // 反向跳变，说明正转
        delta_theta += 2.0f * M_PI;

    last_theta = theta_mech;

    float Speed = delta_theta * f_2kHz * 60.0f * Motor.inv_MotorPn;
    // 电机速度估算 (rpm)
    // delta_theta 单位是 rad
    FOC.Speed = LowPassFilter_Update(&Speed_Filter, Speed);
}

void ClarkeTransform(float_t Ia, float_t Ib, float_t Ic, Clarke_t *out)
{
#if (defined(TWO_PHASE_CURRENT_SENSING))
    out->Ialpha = Ia;
    out->Ibeta = 0.57735026919f * (Ia + 2.0f * Ib); // 0.57735026919f 1/√3
#elif (defined(THREE_PHASE_CURRENT_SENSING))
    out->Ialpha = 0.66666666667f * Ia - 0.33333333333f * (Ib + Ic);
    out->Ibeta = 0.57735026919f * (Ib - Ic); // 0.57735026919f 1/√3
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

static inline float Get_Theta(float Freq, float Theta)
{
    // 电角度递推：θ += ω·Ts，ω = 2π·f
    Theta += 2.0f * M_PI * Freq * T_2kHz;
    if (Theta > 2.0f * M_PI)
        Theta -= 2.0f * M_PI;
    else if (Theta < 0.0f)
        Theta += 2.0f * M_PI;
    return Theta;
}

void SVPWM_Generate(float Ualpha, float Ubeta, float inv_Vdc, float pwm_arr)
{
    uint8_t sector = 0;
    float Vref1 = Ubeta;
    float Vref2 = (+SQRT3 * Ualpha - Ubeta) * 0.5f;
    float Vref3 = (-SQRT3 * Ualpha - Ubeta) * 0.5f;

    // 判断扇区（1~6）
    if (Vref1 > 0)
        sector += 1;
    if (Vref2 > 0)
        sector += 2;
    if (Vref3 > 0)
        sector += 4;

    // Clarke to T1/T2 projection
    // float inv_Vdc = 1.0f / Vdc;
    float X = SQRT3 * Ubeta * inv_Vdc;
    float Y = (+1.5f * Ualpha + SQRT3_2 * Ubeta) * inv_Vdc;
    float Z = (-1.5f * Ualpha + SQRT3_2 * Ubeta) * inv_Vdc;

    float T1 = 0.0f, T2 = 0.0f;

    switch (sector)
    {
    case 1:
        T1 = Z;
        T2 = Y;
        break;
    case 2:
        T1 = Y;
        T2 = -X;
        break;
    case 3:
        T1 = -Z;
        T2 = X;
        break;
    case 4:
        T1 = -X;
        T2 = Z;
        break;
    case 5:
        T1 = X;
        T2 = -Y;
        break;
    case 6:
        T1 = -Y;
        T2 = -Z;
        break;
    default:
        T1 = 0.0f;
        T2 = 0.0f;
        break;
    }

    // 过调制处理
    float T_sum = T1 + T2;
    if (T_sum > 1.0f)
    {
        T1 /= T_sum;
        T2 /= T_sum;
    }

    // 中心对称调制时间计算
    float T0 = (1.0f - T1 - T2) * 0.5f;
    float Ta = T0;
    float Tb = T0 + T1;
    float Tc = Tb + T2;

    float duty_a, duty_b, duty_c;

    // 扇区映射到ABC占空比
    switch (sector)
    {
    case 1:
        duty_a = Tb;
        duty_b = Ta;
        duty_c = Tc;
        break;
    case 2:
        duty_a = Ta;
        duty_b = Tc;
        duty_c = Tb;
        break;
    case 3:
        duty_a = Ta;
        duty_b = Tb;
        duty_c = Tc;
        break;
    case 4:
        duty_a = Tc;
        duty_b = Tb;
        duty_c = Ta;
        break;
    case 5:
        duty_a = Tc;
        duty_b = Ta;
        duty_c = Tb;
        break;
    case 6:
        duty_a = Tb;
        duty_b = Tc;
        duty_c = Ta;
        break;
    default:
        duty_a = 0.5f;
        duty_b = 0.5f;
        duty_c = 0.5f;
        break;
    }

    // 输出PWM占空比（ARR值）
    Set_PWM_Duty(duty_a, duty_b, duty_c, pwm_arr);
}
