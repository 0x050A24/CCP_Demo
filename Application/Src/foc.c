#include "foc.h"
#include "identification.h"
#include "hardware_interface.h"
#include "position_sensor.h"

Motor_Parameter_t Motor;
FOC_Parameter_t FOC;
VF_Parameter_t VF;
IF_Parameter_t IF;
PID_Controller_t Id_PID;
PID_Controller_t Iq_PID;
PID_Controller_t Speed_PID;
RampGenerator_t Speed_Ramp;
InvPark_t Inv_Park;
Clarke_t Clarke;
MTPA_Coefficients_t MTPA;

float theta_mech = 0.0F;
float theta_elec = 0.0F;
float theta_factor = 0.0F;  // Sensor data to mechanic angle conversion factor

float Speed_Ref = 0.0F;
FluxExperiment_t Experiment = {0};

static inline float Get_Theta(float Freq, float Theta);
static inline void Parameter_Init(void);
static inline void Theta_Process(void);
static inline float wrap_theta_2pi(float theta);
static inline float RampGenerator(RampGenerator_t* ramp);
static inline void PID_Controller(float Ref, float Feedback, PID_Controller_t* PID_Controller);
static inline void ClarkeTransform(float_t Ia, float_t Ib, float_t Ic, Clarke_t* out);
static inline void ParkTransform(float_t Ialpha, float_t Ibeta, float_t theta,
                                 FOC_Parameter_t* out);
static inline void InvParkTransform(float_t Udaxis, float_t Uqaxis, float_t theta, InvPark_t* out);
static inline void SVPWM_Generate(float Ualpha, float Ubeta, float inv_Vdc, FOC_Parameter_t* foc);

// SECTION - FOC Main
void FOC_Main(void)
{
  Theta_Process();
  ClarkeTransform(FOC.Ia, FOC.Ib, FOC.Ic, &Clarke);

  switch (FOC.Mode)
  {
    case INIT:
    {
      Parameter_Init();
      FOC.Mode = IDLE;

      Experiment_Init(&Experiment, FOC.Ts, 512, 2, 20, 2, 10, 1, 200);
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
      ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &FOC);
      break;
    }
    // SECTION - IF Mode
    case IF_MODE:
    {
      if (IF.Sensor_State == Enable)
      {
        IF.Theta = FOC.Theta;
      }
      else
      {
        IF.Theta = Get_Theta(IF.IF_Freq, IF.Theta);
      }
      FOC.Theta = IF.Theta;

      ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &FOC);

      FOC.Id_ref = IF.Id_ref;
      FOC.Iq_ref = IF.Iq_ref;

      PID_Controller(FOC.Id_ref, FOC.Id, &Id_PID);
      PID_Controller(FOC.Iq_ref, FOC.Iq, &Iq_PID);

      FOC.Ud_ref = Id_PID.output;
      FOC.Uq_ref = Iq_PID.output;

      break;
    }
    // !SECTION
    // SECTION - Speed Mode
    case Speed:
    {
      ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &FOC);

      static uint16_t Speed_Count = 0;
      Speed_Count++;
      if (Speed_Count > 9)
      {
        Speed_Count = 0;
        Speed_Ramp.target = Speed_Ref;

        PID_Controller(RampGenerator(&Speed_Ramp), FOC.Speed, &Speed_PID);
      }

      FOC.Iq_ref = Speed_PID.output;  // Iq_ref = Speed_PID.output
      FOC.Id_ref = ((MTPA.A * FOC.Iq_ref + MTPA.B) * FOC.Iq_ref + MTPA.C) * FOC.Iq_ref + MTPA.D;

      PID_Controller(FOC.Id_ref, FOC.Id, &Id_PID);
      PID_Controller(FOC.Iq_ref, FOC.Iq, &Iq_PID);

      FOC.Ud_ref = Id_PID.output;
      FOC.Uq_ref = Iq_PID.output;

      break;
    }
    // !SECTION
    case EXIT:
    {
      STOP = 1;
      FOC.Id_ref = 0.0F;  // Id_ref = 0
      FOC.Iq_ref = 0.0F;  // Iq_ref = Speed_PID.output
      FOC.Ud_ref = 0.0F;
      FOC.Uq_ref = 0.0F;
      break;
    }
    // SECTION - Identify Mode
    case Identify:
    {
      ParkTransform(Clarke.Ialpha, Clarke.Ibeta, FOC.Theta, &FOC);

      // //SquareWaveGenerater(&VoltageInjector, &FOC);
      // HighFrequencySquareWaveGenerater(&VoltageInjector);

      // FOC.Ud_ref = VoltageInjector.Vd;
      // FOC.Uq_ref = VoltageInjector.Vq;
      // //FOC.Theta = VoltageInjector.Theta;  // 保持当前 Theta
      if (STOP == 0)
      {
        Experiment_Step(&Experiment, FOC.Id, FOC.Iq, &FOC.Ud_ref, &FOC.Uq_ref);
      }

      break;
    }
    // !SECTION
    default:
    {
      STOP = 1;
      FOC.Mode = IDLE;
      break;
    }
  }

  InvParkTransform(FOC.Ud_ref, FOC.Uq_ref, FOC.Theta, &Inv_Park);
  SVPWM_Generate(Inv_Park.Ualpha, Inv_Park.Ubeta, FOC.inv_Udc, &FOC);
}
// !SECTION

void FOC_UpdateMainFrequency(float f, float Ts, float PWM_ARR)
{
  FOC.f = f;
  FOC.Ts = Ts;
  FOC.PWM_ARR = PWM_ARR;
}
// SECTION - Parameter Init
void Parameter_Init(void)
{
  memset(&VF, 0, sizeof(VF_Parameter_t));
  // memset(&FOC, 0, sizeof(FOC_Parameter_t));
  memset(&Id_PID, 0, sizeof(PID_Controller_t));
  memset(&Iq_PID, 0, sizeof(PID_Controller_t));
  memset(&Speed_PID, 0, sizeof(PID_Controller_t));
  memset(&Inv_Park, 0, sizeof(InvPark_t));
  memset(&Speed_Ramp, 0, sizeof(RampGenerator_t));
  memset(&Motor, 0, sizeof(Motor_Parameter_t));
  memset(&MTPA, 0, sizeof(MTPA_Coefficients_t));

  STOP = 1;

  Motor.Rs = 0.65F;
  Motor.Ld = 0.18F;
  Motor.Lq = 0.12F;
  Motor.Flux = 0.1F;
  Motor.Pn = 2.0F;
  Motor.Position_Scale = 10000 - 1;
  Motor.Resolver_Pn = 1.0F;
  Motor.inv_MotorPn = 1.0F / 2.0F;  // Pn
  Motor.Position_Offset = 107.0F;

  MTPA.A = 0.00061141F;
  MTPA.B = -0.014627F;
  MTPA.C = 0.34737F;
  MTPA.D = 0.068985F;

#ifdef Resolver_Position
  theta_factor = M_2PI / ((Motor.Position_Scale + 1) * Motor.Resolver_Pn);
#endif
#ifdef Encoder_Position
  theta_factor = M_2PI / (float)(Motor.Position_Scale + 1);
#endif
  Speed_PID.Kp = 0.0F;
  Speed_PID.Ki = 0.0F;
  Speed_PID.Kd = 0.0F;
  Speed_PID.MaxOutput = 0.7F * FOC.I_Max;  // Maximum Iq
  Speed_PID.MinOutput = -0.7F * FOC.I_Max;
  Speed_PID.IntegralLimit = 0.7F * FOC.I_Max;
  Speed_PID.previous_error = 0.0F;
  Speed_PID.integral = 0.0F;
  Speed_PID.output = 0.0F;
  Speed_PID.Ts = 10 * FOC.Ts;

  Speed_Ramp.slope = 50.0F;  // limit to 50 rpm/s
  Speed_Ramp.limit_min = -1800.0F;
  Speed_Ramp.limit_max = 1800.0F;
  Speed_Ramp.value = 0.0F;
  Speed_Ramp.target = 0.0F;
  Speed_Ramp.Ts = 10 * FOC.Ts;

  Id_PID.Kp = 73.8274273F;
  Id_PID.Ki = 408.40704496F;
  Id_PID.Kd = 0.0F;
  Id_PID.MaxOutput = 50.0F;  // Maximum Udc/sqrt(3)
  Id_PID.MinOutput = -50.0F;
  Id_PID.IntegralLimit = 50.0F;
  Id_PID.previous_error = 0.0F;
  Id_PID.integral = 0.0F;
  Id_PID.output = 0.0F;
  Id_PID.Ts = FOC.Ts;

  Iq_PID.Kp = 27.646015F;
  Iq_PID.Ki = 408.40704496F;
  Iq_PID.Kd = 0.0F;
  Iq_PID.MaxOutput = 50.0F;
  Iq_PID.MinOutput = -50.0F;
  Iq_PID.IntegralLimit = 50.0F;
  Iq_PID.previous_error = 0.0F;
  Iq_PID.integral = 0.0F;
  Iq_PID.output = 0.0F;
  Iq_PID.Ts = FOC.Ts;

  VF.Vref_Ud = 0.0F;
  VF.Vref_Uq = 0.0F;
  VF.Freq = 0.0F;
  VF.Theta = 0.0F;

  IF.Id_ref = 0.0F;
  IF.Iq_ref = 0.0F;
  IF.IF_Freq = 0.0F;
  IF.Theta = 0.0F;
  IF.Sensor_State = Disable;
}
// !SECTION
// SECTION - PID Controller
static inline void PID_Controller(float Ref, float Feedback, PID_Controller_t* PID_Controller)
{
  float difference = Ref - Feedback;
  float integral = PID_Controller->integral;
  float derivative = difference - PID_Controller->previous_error;

  if (STOP == 1)
  {
    difference = 0.0F;
    integral = 0.0F;
    derivative = 0.0F;
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
  {
    integral = PID_Controller->IntegralLimit;
  }
  else if (integral < -PID_Controller->IntegralLimit)
  {
    integral = -PID_Controller->IntegralLimit;
  }
  // Update integral and previous error for next iteration
  PID_Controller->integral = integral;
  PID_Controller->previous_error = difference;
  // Return the output value
  PID_Controller->output = output_value;
}
// !SECTION

static inline float wrap_theta_2pi(float theta)
{
  if (theta >= M_2PI)
  {
    theta -= M_2PI;
  }
  else if (theta < 0.0F)
  {
    theta += M_2PI;
  }
  return theta;
}

typedef struct
{
  float a;       // 反馈系数（= 极点位置）
  float y_last;  // 上一次输出值
} LowPassFilter_t;

LowPassFilter_t Speed_Filter = {.a = 0.98442F, .y_last = 0.0F};

static inline float LowPassFilter_Update(LowPassFilter_t* filter, float x)
{
  float y = filter->a * filter->y_last + (1.0F - filter->a) * x;
  filter->y_last = y;
  return y;
}

// SECTION - Theta Process
static inline void Theta_Process(void)
{
  int32_t pos_delta = (int32_t)((float)FOC.Position - Motor.Position_Offset);
  if (pos_delta < 0)
  {
    pos_delta += (Motor.Position_Scale + 1);
  }

  theta_mech = (float)pos_delta * theta_factor;

  theta_elec = theta_mech * Motor.Pn;
  FOC.Theta = wrap_theta_2pi(theta_elec);

  static float last_theta = 0.0F;
  float delta_theta = theta_mech - last_theta;

  if (delta_theta > M_PI)
  {
    delta_theta -= M_2PI;
  }
  else if (delta_theta < -M_PI)
  {
    delta_theta += M_2PI;
  }

  last_theta = theta_mech;

  float Speed = delta_theta * FOC.f * 60.0F / M_2PI;
  FOC.Speed = LowPassFilter_Update(&Speed_Filter, Speed);
}
// !SECTION

// SECTION - Ramp Generator
static inline float RampGenerator(RampGenerator_t* ramp)
{
  if (STOP == 1)
  {
    ramp->value = 0.0F;
    ramp->target = 0.0F;
  }

  float delta = ramp->target - ramp->value;
  float step = ramp->slope * ramp->Ts;

  if (delta > step)
  {
    ramp->value += step;
  }
  else if (delta < -step)
  {
    ramp->value -= step;
  }
  else
  {
    ramp->value = ramp->target;
  }

  if (ramp->value > ramp->limit_max)
  {
    ramp->value = ramp->limit_max;
  }
  if (ramp->value < ramp->limit_min)
  {
    ramp->value = ramp->limit_min;
  }

  return ramp->value;
}
// !SECTION

static inline void ClarkeTransform(float_t Ia, float_t Ib, float_t Ic, Clarke_t* out)
{
#if (defined(TWO_PHASE_CURRENT_SENSING))
  out->Ialpha = Ia;
  out->Ibeta = 0.57735026919F * (Ia + 2.0F * Ib);  // 0.57735026919F 1/√3
#elif (defined(THREE_PHASE_CURRENT_SENSING))
  out->Ialpha = 0.66666666667F * Ia - 0.33333333333F * (Ib + Ic);
  out->Ibeta = 0.57735026919F * (Ib - Ic);  // 0.57735026919F 1/√3
#endif
}

static inline void ParkTransform(float_t Ialpha, float_t Ibeta, float_t theta, FOC_Parameter_t* out)
{
  float cos_theta = COS(theta);
  float sin_theta = SIN(theta);
  out->Id = Ialpha * cos_theta + Ibeta * sin_theta;
  out->Iq = -Ialpha * sin_theta + Ibeta * cos_theta;
}

static inline void InvParkTransform(float_t Ud, float_t Uq, float_t theta, InvPark_t* out)
{
  float cos_theta = COS(theta);
  float sin_theta = SIN(theta);
  out->Ualpha = Ud * cos_theta - Uq * sin_theta;
  out->Ubeta = Ud * sin_theta + Uq * cos_theta;
}

static inline float Get_Theta(float Freq, float Theta)
{
  // 电角度递推：θ += ω·Ts，ω = 2π·f
  Theta += M_2PI * Freq * FOC.Ts;
  if (Theta > M_2PI)
  {
    Theta -= M_2PI;
  }
  else if (Theta < 0.0F)
  {
    Theta += M_2PI;
  }
  return Theta;
}

static inline void SVPWM_Generate(float Ualpha, float Ubeta, float inv_Vdc, FOC_Parameter_t* foc)
{
  uint8_t sector = 0;
  float Vref1 = Ubeta;
  float Vref2 = (+SQRT3 * Ualpha - Ubeta) * 0.5F;
  float Vref3 = (-SQRT3 * Ualpha - Ubeta) * 0.5F;

  // 判断扇区（1~6）
  if (Vref1 > 0) sector += 1;
  if (Vref2 > 0) sector += 2;
  if (Vref3 > 0) sector += 4;

  // Clarke to T1/T2 projection
  float X = SQRT3 * Ubeta * inv_Vdc;
  float Y = (+1.5F * Ualpha + SQRT3_2 * Ubeta) * inv_Vdc;
  float Z = (-1.5F * Ualpha + SQRT3_2 * Ubeta) * inv_Vdc;

  float T1 = 0.0F, T2 = 0.0F;

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
      T1 = 0.0F;
      T2 = 0.0F;
      break;
  }

  // 过调制处理
  float T_sum = T1 + T2;
  if (T_sum > 1.0F)
  {
    T1 /= T_sum;
    T2 /= T_sum;
  }

  // 中心对称调制时间计算
  float T0 = (1.0F - T1 - T2) * 0.5F;
  float Ta = T0;
  float Tb = T0 + T1;
  float Tc = Tb + T2;

  float Tcm1 = 0.0F;
  float Tcm2 = 0.0F;
  float Tcm3 = 0.0F;

  // 扇区映射到ABC换相点
  switch (sector)
  {
    case 1:
      Tcm1 = Tb;
      Tcm2 = Ta;
      Tcm3 = Tc;
      break;
    case 2:
      Tcm1 = Ta;
      Tcm2 = Tc;
      Tcm3 = Tb;
      break;
    case 3:
      Tcm1 = Ta;
      Tcm2 = Tb;
      Tcm3 = Tc;
      break;
    case 4:
      Tcm1 = Tc;
      Tcm2 = Tb;
      Tcm3 = Ta;
      break;
    case 5:
      Tcm1 = Tc;
      Tcm2 = Ta;
      Tcm3 = Tb;
      break;
    case 6:
      Tcm1 = Tb;
      Tcm2 = Tc;
      Tcm3 = Ta;
      break;
    default:
      Tcm1 = 0.5F;
      Tcm2 = 0.5F;
      Tcm3 = 0.5F;
      break;
  }

  foc->Tcm1 = Tcm1;
  foc->Tcm2 = Tcm2;
  foc->Tcm3 = Tcm3;
}
