#include "foc_interface.h"
#include <stddef.h>
#include "adc.h"
#include "foc.h"
#include "position_sensor.h"
#include "tim.h"

volatile uint16_t STOP = 1;
ControlStatus Software_BRK = DISABLE;
Protect_Flags Protect_Flag;

float I_Max = 10.0F;

static inline void Current_Protect(float Ia, float Ib, float Ic, float I_Max);

void Interface_GateState(void)
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
    if (gpio_input_bit_get(GPIOE, GPIO_PIN_15) == SET)
    {
      Software_BRK = DISABLE;
      timer_primary_output_config(TIMER0, ENABLE);  // 恢复 MOE
      STOP = 0;
    }
    else
    {
      STOP = 1;
    }
  }
}

void Interface_EnableHardwareProtect(void)
{
  timer_interrupt_enable(TIMER0, TIMER_INT_BRK);  // 启用BRK中断
}

void Interface_DisableHardwareProtect(void)
{
  timer_interrupt_disable(TIMER0, TIMER_INT_BRK);  // 禁用BRK中断
}

// recommended to operate register if possible //
void Interface_SetPWMChangePoint(float Tcm1, float Tcm2, float Tcm3, float pwm_arr)
{
  Set_PWM_Compare(Tcm1, Tcm2, Tcm3, pwm_arr);
}

void Interface_UpdateUdc(void)
{
  float Udc = 0.0F;
  float inv_Udc = 0.0F;
  ADC_Read_Regular(&Udc, &inv_Udc);
  FOC_UpdateVoltage(Udc, inv_Udc);
}

void Interface_UpdateCurrent(void)
{
  float Ia = 0.0F;
  float Ib = 0.0F;
  float Ic = 0.0F;
  ADC_Read_Injection(&Ia, &Ib, &Ic);
  Current_Protect(Ia, Ib, Ic, I_Max);
  FOC_UpdateCurrent(Ia, Ib, Ic);
}

void Interface_UpdatePosition(void)
{
  uint16_t position_data = 0;
  ReadPositionSensor(&position_data);
  FOC_UpdatePosition(position_data);
}

void Interface_CalibrateADC(void)
{
  ADC_Calibration();
}

void Interface_GetSystemFrequency(void)
{
  float f = 0.0F;
  float Ts = 0.0F;
  float PWM_ARR = 0.0F;
  cal_fmain(&f, &Ts, &PWM_ARR);
  FOC_UpdateMainFrequency(f, Ts, PWM_ARR);
}

void Interface_DMASerialSend(void)
{
  USART_DMA_Send_Vofa();
}

static inline void Current_Protect(float Ia, float Ib, float Ic, float I_Max)
{
  if ((Ia > 0.9 * I_Max || Ia < -0.9 * I_Max) || (Ib > 0.9 * I_Max || Ib < -0.9 * I_Max) ||
      (Ic > 0.9 * I_Max || Ic < -0.9 * I_Max))
  {
    static uint16_t Current_Count = 0;
    Current_Count++;
    if (Current_Count > 10)
    {
      STOP = 1;
      Protect_Flag |= Over_Current;
      Current_Count = 0;
    }
  }
  if ((Ia > I_Max || Ia < -1 * I_Max) || (Ib > I_Max || Ib < -1 * I_Max) ||
      (Ic > I_Max || Ic < -1 * I_Max))
  {
    STOP = 1;
    Protect_Flag |= Over_Maximum_Current;
  }
}
