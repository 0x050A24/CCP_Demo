#include "hardware_interface.h"
#include <stddef.h>
#include "adc.h"
#include "can.h"
#include "foc.h"
#include "gpio.h"
#include "position_sensor.h"
#include "stdbool.h"
#include "tim.h"
#include "usart.h"
#include "string.h"

volatile uint16_t STOP = 1;
bool Software_BRK = false;
Protect_Parameter_t Protect = {.Udc_rate = Voltage_Rate,
                               .Udc_fluctuation = Voltage_Fluctuation,
                               .I_Max = Current_Threshold,
                               .Temperature = Temperature_Threshold,
                               .Flag = No_Protect};

static volatile bool usart_dma_busy = false;

static inline void CurrentProtect(float Ia, float Ib, float Ic, float I_Max);
static inline void VoltageProtect(float Udc, float Udc_rate, float Udc_fluctuation);
static inline bool can_receive_to_frame(const can_receive_message_struct* hw_msg,
                                        can_frame_t* frame);

bool Peripheral_CANSend(const can_frame_t* frame)
{
  if (!frame || frame->dlc > 8) return false;

  can_transmit_message_struct tx_msg;
  tx_msg.tx_dlen = frame->dlc;
  memcpy(tx_msg.tx_data, frame->data, frame->dlc);

  tx_msg.tx_ff = frame->is_ext ? CAN_FF_EXTENDED : CAN_FF_STANDARD;
  tx_msg.tx_ft = frame->is_rtr ? CAN_FT_REMOTE : CAN_FT_DATA;

  if (frame->is_ext)
  {
    tx_msg.tx_sfid = 0;  // 无效
    tx_msg.tx_efid = frame->id & 0x1FFFFFFF;
  }
  else
  {
    tx_msg.tx_sfid = frame->id & 0x7FF;
    tx_msg.tx_efid = 0;
  }

  uint8_t mailbox = can_message_transmit(CAN0, &tx_msg);
  return mailbox != CAN_NOMAILBOX;
}

bool Peripheral_CANReceive(can_frame_t* frame)
{
  can_receive_message_struct rx_msg;
  if (CAN_Buffer_Get(&rx_msg))
  {
    if (can_receive_to_frame(&rx_msg, frame))
    {
      return true;
    }
  }
  return false;
}

void Peripheral_SCISend(float* TxBuffer, uint8_t floatnum)
{
  if (usart_dma_busy)
  {
    return;
  }
  USART_DMA_Send(TxBuffer, floatnum);
  usart_dma_busy = true;
}

void Peripheral_SCISendCallback(void)
{
  usart_dma_busy = false;
}

void Peripheral_GateState(void)
{
  if (Protect.Flag != No_Protect)
  {
    STOP = 1;
  }
  if (STOP)
  {
    // 软件触发 BRK
    Software_BRK = true;
    TIMER_SWEVG(TIMER0) |= TIMER_SWEVG_BRKG;
  }
  else
  {
    // STOP = 0，尝试恢复
    if (gpio_input_bit_get(GPIOE, GPIO_PIN_15) == SET)
    {
      Software_BRK = false;
      timer_primary_output_config(TIMER0, ENABLE);  // 恢复 MOE
      STOP = 0;
    }
    else
    {
      STOP = 1;
    }
  }
}

void Peripheral_EnableHardwareProtect(void)
{
  timer_interrupt_enable(TIMER0, TIMER_INT_BRK);  // 启用BRK中断
}

void Peripheral_DisableHardwareProtect(void)
{
  timer_interrupt_disable(TIMER0, TIMER_INT_BRK);  // 禁用BRK中断
}

void Peripheral_InitProtectParameter(void)
{
  Protect.Udc_rate = Voltage_Rate;
  Protect.Udc_fluctuation = Voltage_Fluctuation;
  Protect.Flag = No_Protect;
  // Protect.I_Max = Current_Threshold; Current limit may change during operation
  Protect.Temperature = Temperature_Threshold;
  FOC_UpdateMaxCurrent(Protect.I_Max);
}

// recommended to operate register if possible //
void Peripheral_SetPWMChangePoint(void)
{
  float Tcm1 = 0.0F;
  float Tcm2 = 0.0F;
  float Tcm3 = 0.0F;
  FOC_OutputCompare(&Tcm1, &Tcm2, &Tcm3);
  Set_PWM_Compare(Tcm1, Tcm2, Tcm3);
}

void Peripheral_UpdateUdc(void)
{
  float Udc = 0.0F;
  float inv_Udc = 0.0F;
  ADC_Read_Regular(&Udc, &inv_Udc);
  VoltageProtect(Udc, Protect.Udc_rate, Protect.Udc_fluctuation);
  FOC_UpdateVoltage(Udc, inv_Udc);
}

void Peripheral_UpdateCurrent(void)
{
  float Ia = 0.0F;
  float Ib = 0.0F;
  float Ic = 0.0F;
  ADC_Read_Injection(&Ia, &Ib, &Ic);
  CurrentProtect(Ia, Ib, Ic, Protect.I_Max);
  FOC_UpdateCurrent(Ia, Ib, Ic);
}

void Peripheral_UpdatePosition(void)
{
  uint16_t position_data = 0;
  ReadPositionSensor(&position_data);
  FOC_UpdatePosition(position_data);
}

void Peripheral_CalibrateADC(void)
{
  ADC_Calibration();
}

void Peripheral_GetSystemFrequency(void)
{
  float f = 0.0F;
  float Ts = 0.0F;
  float PWM_ARR = 0.0F;
  cal_fmain(&f, &Ts, &PWM_ARR);
  FOC_UpdateMainFrequency(f, Ts, PWM_ARR);
}

void Peripheral_TemperatureProtect(void)
{
  if (Temperature > 0.375 * Protect.Temperature)
  {
    gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
  }
  if (Temperature < 0.3375 * Protect.Temperature)
  {
    gpio_bit_reset(FAN_OPEN_PORT, FAN_OPEN_PIN);
  
  }
  if (Temperature > Protect.Temperature)
  {
    STOP = 1;
    Protect.Flag |= Over_Heat;
  }
}

static inline void CurrentProtect(float Ia, float Ib, float Ic, float I_Max)
{
  if ((Ia > 0.9 * I_Max || Ia < -0.9 * I_Max) || (Ib > 0.9 * I_Max || Ib < -0.9 * I_Max) ||
      (Ic > 0.9 * I_Max || Ic < -0.9 * I_Max))
  {
    static uint16_t Current_Count = 0;
    Current_Count++;
    if (Current_Count > 10)
    {
      STOP = 1;
      Protect.Flag |= Over_Current;
      Current_Count = 0;
    }
  }
  if ((Ia > I_Max || Ia < -1 * I_Max) || (Ib > I_Max || Ib < -1 * I_Max) ||
      (Ic > I_Max || Ic < -1 * I_Max))
  {
    STOP = 1;
    Protect.Flag |= Over_Maximum_Current;
  }
}

static inline void VoltageProtect(float Udc, float Udc_rate, float Udc_fluctuation)
{
  if ((Udc > Udc_rate + Udc_fluctuation))
  {
    STOP = 1;
    Protect.Flag |= Over_Voltage;
  }
  if ((Udc < Udc_rate - Udc_fluctuation))
  {
    STOP = 1;
    Protect.Flag |= Low_Voltage;
  }
}

static inline bool can_receive_to_frame(const can_receive_message_struct* hw_msg,
                                        can_frame_t* frame)
{
  if (!hw_msg || !frame) return false;

  // 判断标准帧还是扩展帧
  if (hw_msg->rx_ff == CAN_FF_STANDARD)
  {
    frame->id = hw_msg->rx_sfid & 0x7FF;  // 11-bit 标准ID
    frame->is_ext = false;
  }
  else
  {
    frame->id = hw_msg->rx_efid & 0x1FFFFFFF;  // 29-bit 扩展ID
    frame->is_ext = true;
  }

  frame->dlc = (hw_msg->rx_dlen > 8) ? 8 : hw_msg->rx_dlen;
  frame->is_rtr = (hw_msg->rx_ft == CAN_FT_REMOTE);
  memcpy(frame->data, hw_msg->rx_data, frame->dlc);

  return true;
}

