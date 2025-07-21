#include "foc_interface.h"
#include <stddef.h>
#include "adc.h"
#include "can.h"
#include "ccp_interface.h"
#include "gd32f30x.h"  // this header provides ControlStatus which is used by Hardware BRK
#include "position_sensor.h"
#include "tim.h"
#include "usart.h"

#define CAN_TX_BUFFER_SIZE 8

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
} can_tx_message_t;

static can_tx_message_t can_tx_buffer[CAN_TX_BUFFER_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;

volatile uint16_t STOP = 1;
EnableStatus Software_BRK = Disable;
Protect_Parameter_t Protect = {.Udc_rate = Voltage_Rate,
                               .Udc_fluctuation = Voltage_Fluctuation,
                               .I_Max = Current_Threshold,
                               .Temperature = Temperature_Threshold,
                               .Flag = No_Protect};

static inline void CurrentProtect(float Ia, float Ib, float Ic, float I_Max);
static inline void VoltageProtect(float Udc, float Udc_rate, float Udc_fluctuation);
static inline bool CAN_to_CCP(const can_receive_message_struct* can_msg,
                                     ccp_message_t* ccp_msg);

bool Interface_ReadCANMessage(can_rx_message_t* out_msg)
{
    can_receive_message_struct can_msg;

    if (CAN_Buffer_Get(&can_msg))
    {
        // 判断是否是CCP
        ccp_message_t ccp_msg;
        if (CAN_to_CCP(&can_msg, &ccp_msg))
        {
            out_msg->Protocol = CCP;
            out_msg->msg.ccp_msg = ccp_msg;
            return true;
        }
        // 其他协议判断...
        // 未识别消息，丢弃或者统计
    }

    return false;
}

bool Interface_CANTXEnqueue(uint32_t id, const uint8_t *data, size_t len)
{
    if (!data || len > 8) return false;

    uint8_t next_head = (tx_head + 1) % CAN_TX_BUFFER_SIZE;
    if (next_head == tx_tail) {
        // 缓冲区满
        return false;
    }

    can_tx_buffer[tx_head].id = id;
    can_tx_buffer[tx_head].len = len;
    memcpy(can_tx_buffer[tx_head].data, data, len);
    tx_head = next_head;

    return true;
}

void Interface_CANTXProcess(void)
{
    while (tx_tail != tx_head)
    {
        can_tx_message_t *msg = &can_tx_buffer[tx_tail];

        can_transmit_message_struct tx_msg;
        tx_msg.tx_sfid = msg->id;
        tx_msg.tx_efid = 0;
        tx_msg.tx_ff = CAN_FF_STANDARD;
        tx_msg.tx_ft = CAN_FT_DATA;
        tx_msg.tx_dlen = msg->len;
        memcpy(tx_msg.tx_data, msg->data, msg->len);

        uint8_t mailbox = can_message_transmit(CAN0, &tx_msg);

        if (mailbox == CAN_NOMAILBOX)
        {
            // 无邮箱空闲，稍后重试（保留当前数据）
            break;
        }
        // Only if successfully sent, move the tail
        tx_tail = (tx_tail + 1) % CAN_TX_BUFFER_SIZE;
    }
}

static inline bool CAN_to_CCP(const can_receive_message_struct* can_msg,
                                     ccp_message_t* ccp_msg)
{
  if (!can_msg || !ccp_msg)
  {
    return false;
  }

  uint32_t id = (can_msg->rx_ff == CAN_FF_STANDARD) ? can_msg->rx_sfid : can_msg->rx_efid;
  if (id != CCP_CRO_ID || can_msg->rx_dlen != 8)
  {
    return false;
  }

  memcpy(ccp_msg->data, can_msg->rx_data, 8);
  return true;
}

void Interface_GateState(void)
{
  if (Protect.Flag != No_Protect)
  {
    STOP = 1;
  }
  if (STOP)
  {
    // 软件触发 BRK
    Software_BRK = Enable;
    TIMER_SWEVG(TIMER0) |= TIMER_SWEVG_BRKG;
  }
  else
  {
    // STOP = 0，尝试恢复
    if (gpio_input_bit_get(GPIOE, GPIO_PIN_15) == SET)
    {
      Software_BRK = Disable;
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

void Interface_InitProtectParameter(void)
{
  Protect.Udc_rate = Voltage_Rate;
  Protect.Udc_fluctuation = Voltage_Fluctuation;
  Protect.Flag = No_Protect;
  // Protect.I_Max = Current_Threshold; Current limit may change during operation
  Protect.Temperature = Temperature_Threshold;
  FOC_UpdateMaxCurrent(Protect.I_Max);
}

// recommended to operate register if possible //
void Interface_SetPWMChangePoint(void)
{
  float Tcm1 = 0.0F;
  float Tcm2 = 0.0F;
  float Tcm3 = 0.0F;
  FOC_OutputCompare(&Tcm1, &Tcm2, &Tcm3);
  Set_PWM_Compare(Tcm1, Tcm2, Tcm3);
}

void Interface_UpdateUdc(void)
{
  float Udc = 0.0F;
  float inv_Udc = 0.0F;
  ADC_Read_Regular(&Udc, &inv_Udc);
  VoltageProtect(Udc, Protect.Udc_rate, Protect.Udc_fluctuation);
  FOC_UpdateVoltage(Udc, inv_Udc);
}

void Interface_UpdateCurrent(void)
{
  float Ia = 0.0F;
  float Ib = 0.0F;
  float Ic = 0.0F;
  ADC_Read_Injection(&Ia, &Ib, &Ic);
  CurrentProtect(Ia, Ib, Ic, Protect.I_Max);
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

void Interface_DMASerialSend(float* TxBuffer, uint16_t DataSize)
{
  USART_DMA_Send_Vofa(TxBuffer, DataSize);
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
