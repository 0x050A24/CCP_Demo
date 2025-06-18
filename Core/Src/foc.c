
#include "foc.h"

uint16_t STOP = 1;

volatile Protect_Flags Protect_Flag;
FOC_Parameter_t FOC;
VF_Parameter_t VF;
IF_Parameter_t IF;
PID_Controller_t Id_PID;
PID_Controller_t Iq_PID;
InvPark_t Inv_Park;
Clarke_t Clarke;
Park_t Park;

float I_Max = 10.0f;

static inline void Current_Protect(void);
static inline void Get_Theta(float freq);
static inline void Parameter_Init(void);

void FOC_Main(void)
{
    ADC_Read_Injection();
    Current_Protect();
    Gate_state();

    switch (FOC.Mode)
    {
    case INIT:
    {
        Parameter_Init();
        if (Udc > 200.0f || inv_Udc < 0.005f) // 200V
        {
            ADC_Calibration();
            FOC.Mode = IDLE;
        }
        break;
    }
    case IDLE:
    {
        STOP = 1;
        break;
    }
    case VF_MODE:
    {
        Get_Theta(VF.Freq);
        InvParkTransform(VF.Vref_Ud, VF.Vref_Uq, FOC.theta, &Inv_Park);
        SVPWM_Generate(Inv_Park.Ualpha, Inv_Park.Ubeta, inv_Udc, FOC.pwm_arr);
        break;
    }
    case IF_MODE:
    {
        ClarkeTransform(Ia, Ib, Ic, &Clarke);
        Get_Theta(IF.IF_Freq);
        ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.theta, &Park);

        PID_Controller(IF.Id_ref, Park.Id, &Id_PID);
        PID_Controller(IF.Iq_ref, Park.Iq, &Iq_PID);
        
        InvParkTransform(Id_PID.output, Iq_PID.output, FOC.theta, &Inv_Park);
        SVPWM_Generate(Inv_Park.Ualpha, Inv_Park.Ubeta, inv_Udc, FOC.pwm_arr);
        break;
    }
    default:
    {
        // 其他模式处理
        timer_interrupt_enable(TIMER0, TIMER_INT_BRK); // 启用BRK中断
        break;
    }
    }
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
        TIMER_SWEVG(TIMER0) |= TIMER_SWEVG_BRKG;
    }
    else
    {
        // STOP = 0，尝试恢复
        //if (gpio_input_bit_get(GPIOE, GPIO_PIN_15) == RESET) 
        {
            timer_primary_output_config(TIMER0, ENABLE); // 恢复 MOE
            STOP = 0;
        }
        // else
        // {
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

void Parameter_Init(void)
{
    memset(&VF, 0, sizeof(VF_Parameter_t));
    memset(&FOC, 0, sizeof(FOC_Parameter_t));
    memset(&Id_PID, 0, sizeof(PID_Controller_t));
    memset(&Iq_PID, 0, sizeof(PID_Controller_t));
    memset(&Inv_Park, 0, sizeof(InvPark_t));

    Protect_Flag = No_Protect;

    Id_PID.Kp = 0.1f;
    Id_PID.Ki = 0.01f;
    Id_PID.Kd = 0.0f;
    Id_PID.MaxOutput = 5.0f; // Maximum Udc/sqrt(3)
    Id_PID.MinOutput = -5.0f;
    Id_PID.IntegralLimit = 5.0f;
    Id_PID.previous_error = 0.0f;
    Id_PID.integral = 0.0f;
    Id_PID.output = 0.0f;
    Id_PID.Ts = T_2Khz;
    
    Iq_PID.Kp = 0.1f;
    Iq_PID.Ki = 0.01f;
    Iq_PID.Kd = 0.0f;
    Iq_PID.MaxOutput = 5.0f;
    Iq_PID.MinOutput = -5.0f;
    Iq_PID.IntegralLimit = 5.0f;
    Iq_PID.previous_error = 0.0f;
    Iq_PID.integral = 0.0f;
    Iq_PID.output = 0.0f;
    Iq_PID.Ts = T_2Khz;

    FOC.pwm_arr = PWM_ARR;
}

void PID_Controller(float setpoint, float measured_value, PID_Controller_t *PID_Controller)
{
    float difference = setpoint - measured_value;
    float integral = PID_Controller->integral;
    float derivative = difference - PID_Controller->previous_error;

    if(STOP == 1)
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

static inline void Get_Theta(float freq)
{
    // 电角度递推：θ += ω·Ts，ω = 2π·f
    FOC.theta += 2.0f * M_PI * freq * T_2Khz;
    if (FOC.theta > 2.0f * M_PI)
        FOC.theta -= 2.0f * M_PI;
    else if (FOC.theta < 0.0f)
        FOC.theta += 2.0f * M_PI;
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
