#include "foc.h"
#include "buffer.h"
#include "leso.h"
#include "pid.h"
#include "signal.h"
#include "stdint.h"
#include "transformation.h"

#include "MTPA.h"
#include "identification.h"

static FocMode_t Foc_Mode            = IDLE;   // 当前FOC模式
static FocMode_t Foc_Mode_Prev       = IDLE;   // 上一次FOC模式
static bool      Foc_Reset           = false;  // FOC复位标志
static float     Foc_Current_Ts      = 0.0F;   // 电流环采样周期
static float     Foc_Current_Freq    = 0.0F;   // 电流环频率
static uint16_t  Foc_Speed_Prescaler = 0U;     // 电流环分频数
static float     Foc_Speed_Ts        = 0.0F;   // 转速环采样周期
static float     Foc_Speed_Freq      = 0.0F;   // 转速环频率
static float     Foc_Speed_Ref       = 0.0F;   // 参考速度
static float     Foc_Speed_Fdbk      = 0.0F;   // 实际转速反馈
static float     Foc_Theta           = 0.0F;
static float     Foc_BusVoltage      = 0.0F;
static float     Foc_BusVoltage_Inv  = 0.0F;
static float     Foc_Speed_Ramp      = 0.0F;  // 实际指令转速

static volatile float Foc_Id_Min = 0.3F;  // D轴电流最小值
static volatile bool  Foc_Sweep  = true;  // FOC扫频标志

static VF_Parameter_t  Foc_VfParam            = {0};
static IF_Parameter_t  Foc_IfParam            = {0};
static Clark_t         Foc_Iclark_Fdbk        = {0};
static Park_t          Foc_Idq_Ref            = {0};
static Park_t          Foc_Idq_Fdbk           = {0};
static Clark_t         Foc_Uclark_Ref         = {0};
static Park_t          Foc_Udq_Ref            = {0};
static PID_Handler_t   Foc_Pid_Speed_Handler  = {0};
static PID_Handler_t   Foc_Pid_CurD_Handler   = {0};
static PID_Handler_t   Foc_Pid_CurQ_Handler   = {0};
static RampGenerator_t Foc_Ramp_Speed_Handler = {0};
static SawtoothWave_t  Foc_Sawtooth_Handler   = {0};

IIR1stFilter_t Leso_EMF_Filter_Fast     = {0};
IIR1stFilter_t Leso_EMF_Filter_Slow     = {0};
MovingAvg_t    Leso_EMF_MovingAvg       = {0};
MovingAvg_t    Leso_EMF_MovingAvg_Short = {0};

static PI_Tuner_t Foc_Pi_Tuner
    = {.Tune_Ratio = 5.0F, .Tune_Threshold = 50.0F, .Tuned = false};

FluxExperiment_t Experiment = {0};

float_t Comp_Max = 20.0F;

static inline void Compensate_Trigger(float_t slow, float_t fast);
static inline void Compensate_Update(float_t error, float_t sign);

void Foc_Set_SampleTime(const SystemTimeConfig_t* config)
{
    Foc_Current_Ts      = config->current.val;  // 电流环采样周期
    Foc_Current_Freq    = config->current.inv;  // 电流环频率
    Foc_Speed_Ts        = config->speed.val;    // 转速环采样周期
    Foc_Speed_Freq      = config->speed.inv;    // 转速环频率
    Foc_Speed_Prescaler = (uint16_t)config->prescaler;  // 转速环分频数
}

void Foc_Set_Mode(FocMode_t mode)
{
    Foc_Mode = mode;  // 设置FOC模式
}

FocMode_t Foc_Get_Mode(void)
{
    return Foc_Mode;  // 获取FOC模式
}

void Foc_Set_ResetFlag(bool reset)
{
    Foc_Reset = reset;  // 设置复位标志
}

bool Foc_Get_ResetFlag(void)
{
    if (Foc_Mode == IDLE)
    {
        return true;  // 在IDLE模式下始终返回true
    }
    return Foc_Reset;  // 获取复位标志状态
}

// bool Foc_Get_ResetFlag(void) {
//     if (Foc_Mode_Prev != Foc_Mode) {
//         // 防止意外切换模式
//         Foc_Mode_Prev = Foc_Mode;
//         return true;
//     }
//     if (Foc_Mode == VF_MODE || Foc_Mode == IF_MODE
//         || Foc_Mode == SPEED) {
//         return Foc_Reset;  // 获取复位标志状态
//     }
//     return true;  // 在IDLE模式下始终返回true
// }

void Foc_Set_Angle(float angle)
{
    Foc_Theta = wrap_theta_2pi(angle);  // 确保角度在 [0, 2π) 范围内
}

void Foc_Set_BusVoltage(float voltage)
{
    Foc_BusVoltage = voltage;  // 设置母线电压
}

float Foc_Get_BusVoltage(void)
{
    return Foc_BusVoltage;  // 获取母线电压
}

Park_t Foc_Get_Inductor(void)
{
    return Mtpa_Get_LPark();  // 获取电感
}

void Foc_Set_BusVoltageInv(float voltage)
{
    Foc_BusVoltage_Inv = voltage;  // 设置母线电压倒数
}

void Foc_Set_Speed(float speed)
{
    Foc_Speed_Fdbk = speed;  // 设置参考速度
}

float Foc_Get_SpeedRamp(void)
{
    return Foc_Speed_Ramp;
}

float Foc_Get_SpeedTarget(void)
{
    return Foc_Speed_Ref;
}

void Foc_Set_Speed_and_Angle(AngleResult_t* angle_speed)
{
    Foc_Theta      = wrap_theta_2pi(angle_speed->theta);
    Foc_Speed_Fdbk = angle_speed->speed;
}

void Foc_Set_Iclark_Fdbk(Clark_t current)
{
    Foc_Iclark_Fdbk = current;  // 设置电流反馈
}

Clark_t Foc_Get_Iclark_Fdbk(void)
{
    return Foc_Iclark_Fdbk;  // 获取αβ轴电流反馈
}

void Foc_Set_Idq_Ref(Park_t idq_ref)
{
    Foc_Idq_Ref = idq_ref;  // 设置DQ轴电流参考
}

Park_t Foc_Get_Idq_Ref(void)
{
    return Foc_Idq_Ref;  // 获取DQ轴电流参考
}

void Foc_Set_Idq_Fdbk(Park_t idq_fdbk)
{
    Foc_Idq_Fdbk = idq_fdbk;  // 设置DQ轴电流反馈
}

Park_t Foc_Get_Idq_Fdbk(void)
{
    return Foc_Idq_Fdbk;  // 获取DQ轴电流反馈
}

void Foc_Set_Udq_Ref(Park_t udq_ref)
{
    Foc_Udq_Ref = udq_ref;  // 设置DQ轴电压参考

    // 将DQ轴电压参考转换为αβ轴电压参考
    Foc_Uclark_Ref = InvParkTransform(Foc_Udq_Ref, Foc_Theta);
}

Park_t Foc_Get_Udq_Ref(void)
{
    return Foc_Udq_Ref;  // 获取DQ轴电压参考
}

void Foc_Set_Uclark_Ref(Clark_t uclark_ref)
{
    Foc_Uclark_Ref = uclark_ref;  // 设置αβ轴电压参考
}

Clark_t Foc_Get_Uclark_Ref(void)
{
    return Foc_Uclark_Ref;  // 获取αβ轴电压参考
}

void Foc_Set_Vf_Param(VF_Parameter_t* vf_param)
{
    Foc_VfParam = *vf_param;  // 设置VF参数
}

void Foc_Set_If_Param(IF_Parameter_t* if_param)
{
    Foc_IfParam = *if_param;  // 设置IF参数
}

void Foc_Set_Pid_Speed_Handler(PID_Handler_t* handler)
{
    Foc_Pid_Speed_Handler = *handler;  // 设置速度环PID控制器
}

void Foc_Set_Pid_CurD_Handler(PID_Handler_t* handler)
{
    Foc_Pid_CurD_Handler = *handler;  // 设置D轴电流环PID控制器
}

void Foc_Set_Pid_CurQ_Handler(PID_Handler_t* handler)
{
    Foc_Pid_CurQ_Handler = *handler;  // 设置Q轴电流环PID控制器
}

void Foc_Set_Ramp_Speed_Handler(RampGenerator_t* handler)
{
    Foc_Ramp_Speed_Handler = *handler;  // 设置速度环斜坡生成器
}

static inline Phase_t calculate_SVPWM_Tcm(Clark_t u_ref, float inv_Vdc)
{
    float   alpha  = u_ref.a;
    float   beta   = u_ref.b;
    uint8_t sector = 0;
    float   v_ref1 = beta;
    float   v_ref2 = (+SQRT3 * alpha - beta) * 0.5F;
    float   v_ref3 = (-SQRT3 * alpha - beta) * 0.5F;

    // 判断扇区（1~6）
    if (v_ref1 > 0)
    {
        sector += 1;
    }
    if (v_ref2 > 0)
    {
        sector += 2;
    }
    if (v_ref3 > 0)
    {
        sector += 4;
    }

    // Clarke to t1/t2 projection
    float X = SQRT3 * beta * inv_Vdc;
    float Y = (+1.5F * alpha + SQRT3_2 * beta) * inv_Vdc;
    float Z = (-1.5F * alpha + SQRT3_2 * beta) * inv_Vdc;

    float t1 = 0.0F, t2 = 0.0F;

    switch (sector)
    {
    case 1:
        t1 = Z;
        t2 = Y;
        break;
    case 2:
        t1 = Y;
        t2 = -X;
        break;
    case 3:
        t1 = -Z;
        t2 = X;
        break;
    case 4:
        t1 = -X;
        t2 = Z;
        break;
    case 5:
        t1 = X;
        t2 = -Y;
        break;
    case 6:
        t1 = -Y;
        t2 = -Z;
        break;
    default:
        t1 = 0.0F;
        t2 = 0.0F;
        break;
    }

    // 过调制处理
    float T_sum = t1 + t2;
    if (T_sum > 1.0F)
    {
        t1 /= T_sum;
        t2 /= T_sum;
    }

    // 中心对称调制时间计算
    float t0 = (1.0F - t1 - t2) * 0.5F;
    float ta = t0;
    float tb = t0 + t1;
    float tc = tb + t2;

    Phase_t tcm = {0.0F, 0.0F, 0.0F};

    // 扇区映射到ABC换相点
    switch (sector)
    {
    case 1:
        tcm.a = tb;
        tcm.b = ta;
        tcm.c = tc;
        break;
    case 2:
        tcm.a = ta;
        tcm.b = tc;
        tcm.c = tb;
        break;
    case 3:
        tcm.a = ta;
        tcm.b = tb;
        tcm.c = tc;
        break;
    case 4:
        tcm.a = tc;
        tcm.b = tb;
        tcm.c = ta;
        break;
    case 5:
        tcm.a = tc;
        tcm.b = ta;
        tcm.c = tb;
        break;
    case 6:
        tcm.a = tb;
        tcm.b = tc;
        tcm.c = ta;
        break;
    default:
        tcm.a = 0.5F;
        tcm.b = 0.5F;
        tcm.c = 0.5F;
        break;
    }

    return tcm;
}

Phase_t Foc_Get_Tcm(void)
{
    Phase_t tcm = {.a = 0.5F, .b = 0.5F, .c = 0.5F};
    if (Foc_Reset)
    {
        return tcm;  // 如果复位标志为真，直接返回零值
    }
    // 生成三相PWM时间
    tcm = calculate_SVPWM_Tcm(Foc_Uclark_Ref, Foc_BusVoltage_Inv);

    return tcm;  // 返回三相PWM时间
}

static inline float dispatch_current(float cur_ref)
{
    if (cur_ref < 0.0F)
    {
        cur_ref = -cur_ref;
    }
    // float out = 0.0003497F * cur_ref * cur_ref * cur_ref - 0.02016F * cur_ref * cur_ref +
    //             0.7335F * cur_ref + 0.6032F;  // 三次函数拟合
    float out = 0.0F;
    MTPA_interp_by_Iq(
        mtpa_table, MTPA_TABLE_POINTS, cur_ref, &out, NULL);
    return out;
}

static inline Park_t Foc_Update_SpeedLoop(float ref,
                                          float fdbk,
                                          bool  reset)
{
    static uint16_t counter = 0x0000U;
    counter++;
    if (counter < Foc_Speed_Prescaler)
    {
        return Foc_Idq_Ref;  // 如果未到达分频点，直接返回参考值
    }
    counter                       = 0x0000U;
    Foc_Ramp_Speed_Handler.target = ref;  // 更新目标速度
    Park_t output                 = {0};
    float  ramp    = RampGenerator(&Foc_Ramp_Speed_Handler, reset);
    Foc_Speed_Ramp = ramp;
    output.q = Pid_Update(ramp - fdbk, reset, &Foc_Pid_Speed_Handler);
#if defined(FOC_DEBUG_IQ)
    static float iqtest = 0;
    iqtest              = iqtest + 0.0002F;
    if (iqtest > 21.0F)
    {
        iqtest = 0.0F;
    }
    output.q = iqtest;
#endif
    output.d = dispatch_current(output.q);

    return output;  // 返回DQ轴电流参考
}

static inline Park_t Foc_Update_CurrentLoop(Park_t ref,
                                            Park_t fdbk,
                                            bool   reset)
{
    Park_t output = {0};

    Pid_Update(ref.d - fdbk.d, reset, &Foc_Pid_CurD_Handler);
    Pid_Update(ref.q - fdbk.q, reset, &Foc_Pid_CurQ_Handler);

    output.d = Foc_Pid_CurD_Handler.output;
    output.q = Foc_Pid_CurQ_Handler.output;

    return output;
}

static inline Park_t Foc_Update_VfMode(bool reset)
{
    static bool  reset_prev = true;
    static bool  sweep_flag = false;
    static float phase_prev = 0.0F;
    Park_t       output     = {0};
    if (reset_prev && !reset)
    {
        SawtoothWave_Init(&Foc_Sawtooth_Handler,
                          M_2PI,
                          Foc_VfParam.freq,
                          0.0F,
                          Foc_Current_Ts);
    }
    Foc_Sawtooth_Handler.frequency = Foc_VfParam.freq;
    output = Foc_VfParam.vol_ref;  // 获取电压参考

    float phase = 0.0F;
    phase       = SawtoothWaveGenerator(&Foc_Sawtooth_Handler,
                                  reset);  // 更新电压环角度
    if (fabsf(phase - phase_prev) > M_PI_2)
    {
        sweep_flag = false;
    }
    if (Foc_Sweep)
    {
        sweep_flag = true;
    }
    if (sweep_flag)
    {
        phase_prev = phase;
    }
    Foc_Theta = wrap_theta_2pi(phase_prev + Foc_VfParam.offset);

    Foc_Idq_Fdbk = ParkTransform(Foc_Iclark_Fdbk, Foc_Theta);
    reset_prev   = reset;
    return output;
}

static inline Park_t Foc_Update_IfMode(bool reset)
{
    static bool  reset_prev = true;
    static bool  sweep_flag = false;
    static float phase_prev = 0.0F;
    Park_t       output     = {0};
    // 如果传感器状态为启用，则直接使用参考值，否则使用正弦波生成器
    if (Foc_IfParam.use_sensor == true)
    {
        Foc_IfParam.offset = 0.0F;
    }
    else
    {
        if (reset_prev && !reset)
        {
            SawtoothWave_Init(&Foc_Sawtooth_Handler,
                              M_2PI,
                              Foc_IfParam.freq,
                              0.0F,
                              Foc_Current_Ts);
        }
        Foc_Sawtooth_Handler.frequency = Foc_IfParam.freq;

        float phase = 0.0F;
        phase       = SawtoothWaveGenerator(&Foc_Sawtooth_Handler,
                                      reset);  // 更新电流环角度
        if (fabsf(phase - phase_prev) > M_PI_2)
        {
            sweep_flag = false;
        }
        if (Foc_Sweep)
        {
            sweep_flag = true;
        }
        if (sweep_flag)
        {
            phase_prev = phase;
        }

        Foc_Theta = wrap_theta_2pi(phase_prev + Foc_IfParam.offset);
    }

    Foc_Idq_Fdbk = ParkTransform(Foc_Iclark_Fdbk, Foc_Theta);

    output = Foc_Update_CurrentLoop(
        Foc_IfParam.cur_ref, Foc_Idq_Fdbk, reset);

    reset_prev = reset;
    return output;
}

static inline Park_t Foc_Update_SpeedMode(bool reset)
{
    if (reset)
    {
        // 对Foc_Speed_Ref进行一次写入操作，防止变量被优化掉
        Foc_Speed_Ref = 0.0F;
    }

    Foc_Idq_Fdbk = ParkTransform(Foc_Iclark_Fdbk, Foc_Theta);

    Park_t output       = {0};
    Park_t Foc_Idq_Comp = {0};
    // 更新转速环
    Foc_Idq_Ref
        = Foc_Update_SpeedLoop(Foc_Speed_Ref, Foc_Speed_Fdbk, reset);

    output = Foc_Update_CurrentLoop(Foc_Idq_Ref, Foc_Idq_Fdbk, reset);

    if (Leso_Enabled)
    {
        Leso_EmfEst_dq      = ParkTransform(Leso_EmfEst, Foc_Theta);
        Leso_Emf_Filtered.q = IIR1stFilter_Update(&Leso_EMF_Filter_Fast,
                                                  Leso_EmfEst_dq.q);
        Leso_Emf_Filtered.d = IIR1stFilter_Update(&Leso_EMF_Filter_Fast,
                                                  Leso_EmfEst_dq.d);

        Leso_Emf_Filtered.q = MovingAvg_Update(
            &Leso_EMF_MovingAvg_Short, Leso_EmfEst_dq.q);
        Leso_Emf_Slow_Filtered.q
            = MovingAvg_Update(&Leso_EMF_MovingAvg, Leso_EmfEst_dq.q);

        float error = fabsf(Leso_Emf_Slow_Filtered.q - Comp.trigger_on);
        float sign
            = (Leso_Emf_Slow_Filtered.q > EMF_TRIG_MIN) ? -1.0f : +1.0f;
        Compensate_Trigger(Leso_Emf_Slow_Filtered.q,
                           Leso_Emf_Filtered.q);
        Compensate_Update(error, sign);

        if (Comp.active)
        {
            // Foc_Idq_Comp.q = sign * Comp.value;
            // Foc_Idq_Ref.q += Foc_Idq_Comp.q;
            //output.q += sign * Comp.value;
        }
    }

    if ((fabsf(Foc_Speed_Fdbk - Foc_Speed_Ramp)
         > Foc_Pi_Tuner.Tune_Threshold)
        && Foc_Pi_Tuner.Tuned == false)
    {
        Foc_Pid_Speed_Handler.Kp
            = Foc_Pi_Tuner.Tune_Ratio * Foc_Pid_Speed_Handler.Kp;
        Foc_Pid_Speed_Handler.Ki
            = Foc_Pi_Tuner.Tune_Ratio * Foc_Pid_Speed_Handler.Ki;
        Foc_Pi_Tuner.Tuned = true;
    }
    if (Foc_Pi_Tuner.Tuned)
    {
        if (fabsf(Foc_Speed_Fdbk - Foc_Speed_Ramp)
            < Foc_Pi_Tuner.Tune_Threshold)
        {
            Foc_Pid_Speed_Handler.Kp
                = Foc_Pid_Speed_Handler.Kp / Foc_Pi_Tuner.Tune_Ratio;
            Foc_Pid_Speed_Handler.Ki
                = Foc_Pid_Speed_Handler.Ki / Foc_Pi_Tuner.Tune_Ratio;
            Foc_Pi_Tuner.Tuned = false;
        }
    }

    // 更新电流环
    if (output.q > PID_CURRENT_Q_LOOP_MAX_OUTPUT)
    {
        output.q = PID_CURRENT_Q_LOOP_MAX_OUTPUT;
    }
    if (output.q < -PID_CURRENT_Q_LOOP_MAX_OUTPUT)
    {
        output.q = -PID_CURRENT_Q_LOOP_MAX_OUTPUT;
    }

    Buffer_Put(output.q, 5);
    Buffer_Put(Foc_Idq_Fdbk.q, 6);
    Buffer_Put(Comp.value, 7);

    return output;
}

static inline void Compensate_Trigger(float_t slow, float_t fast)
{
    float error     = fabsf(fast - slow);
    Comp.trigger_on = K_TRIG * slow;
    if (Comp.trigger_on < EMF_TRIG_MIN)
    {
        Comp.trigger_on = EMF_TRIG_MIN;
    }
    // Comp.trigger_off = 0.5f * Comp.trigger_on;
    Comp.trigger_off = 250.0F;

    if (!Comp.active)
    {
        // if (error > Comp.trigger_on)
        if (slow > Comp.trigger_on)
        {
            Comp.active = true;
        }
    }
    else
    {
        if (slow < Comp.trigger_off)
        {
            Comp.active = false;
        }
    }
}

static inline void Compensate_Update(float_t error, float_t sign)
{
    // 上升沿：刚触发
    if (Comp.active && !Comp.active_last)
    {
        // Comp.init  = K_COMP * fabsf(error);
        Comp.init  = Comp_Max;
        Comp.init  = (Comp.init < 0.0f)       ? 0.0f
                     : (Comp.init > Comp_Max) ? Comp_Max
                                              : Comp.init;
        Comp.value = Comp.init;
        Comp.ticks = BASE_COMP_TICKS + K_TICKS * Comp.init;
    }

    if (Comp.active)
    {
        Comp.value *= COMP_ALPHA;
        if (--Comp.ticks <= 0)
        {
            Comp.active = false;
            Comp.value  = 0.0f;
        }
    }
    else
    {
        Comp.value = 0.0f;
    }

    Comp.active_last = Comp.active;
}

Park_t Foc_Update_Main(void)
{
    Park_t output = {0};

    switch (Foc_Mode)
    {
    case VF_MODE:
    {
        output = Foc_Update_VfMode(Foc_Reset);  // VF模式
        break;
    }
    case IF_MODE:
    {
        output = Foc_Update_IfMode(Foc_Reset);  // IF模式
        break;
    }
    case STARTUP:
    {
        output.d = 5.0F;  // D轴电压参考为5
        output.q = 0.0F;  // Q轴电压参考为0
        break;
    }
    case SPEED:
    {
        output = Foc_Update_SpeedMode(Foc_Reset);  // 转速模式
        break;
    }
    case IDENTIFY:
    {
        if (Experiment.Initialized == false)
        {
            Experiment_Init(&Experiment,
                            Foc_Current_Ts,
                            SAMPLE_CAPACITY,
                            REPEAT_TIMES,
                            MAX_STEPS,
                            3,
                            12,
                            1,
                            100);
            MTPA_build_table(
                mtpa_table, MTPA_TABLE_POINTS, 0.0f, 50.0f);
        }
        else if (Experiment.Complete == true)
        {
            float ad0 = 0.0F, add = 0.0F, aq0 = 0.0F, aqq = 0.0F,
                  adq = 0.0F;
            Get_Identification_Results(
                &Experiment, &ad0, &add, &aq0, &aqq, &adq);
            MTPA_Get_Parameter(ad0, add, aq0, aqq, adq);
            MTPA_build_table(
                mtpa_table, MTPA_TABLE_POINTS, 0.0f, 50.0f);
            Foc_Mode = IDLE;
            break;
        }
        Foc_Idq_Fdbk = ParkTransform(Foc_Iclark_Fdbk, Foc_Theta);
        Experiment_Step(&Experiment,
                        Foc_Idq_Fdbk.d,
                        Foc_Idq_Fdbk.q,
                        &output.d,
                        &output.q);
        break;
    }
    default:
    {
        output.d = 0.0F;  // 默认情况下，D轴电压参考为0
        output.q = 0.0F;  // Q轴电压参考为0

        break;
    }
    }
    Foc_Udq_Ref    = output;
    Foc_Uclark_Ref = InvParkTransform(Foc_Udq_Ref, Foc_Theta);
    return output;  // 返回DQ轴电压参考
}
