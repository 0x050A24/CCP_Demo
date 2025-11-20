#include "hardware_interface.h"
#include <stdbool.h>
#include <stddef.h>
#include "adc.h"
#include "can.h"
#include "gpio.h"
#include "motor.h"
#include "position_sensor.h"
#include "protect.h"
#include "reciprocal.h"
#include "theta_calc.h"
#include "tim.h"
#include "transformation.h"
#include "usart.h"

static volatile uint16_t Stop           = 0x0001U;
static bool              Software_BRK   = false;
static volatile bool     usart_dma_busy = false;
volatile float           Breke_Duty     = 1.0F;

static inline bool can_receive_to_frame(
    const can_receive_message_struct* hw_msg, can_frame_t* frame);

bool Peripheral_CANSend(const can_frame_t* frame)
{
    if (!frame || frame->dlc > 8)
    {
        return false;
    }

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

bool Peripheral_Get_SoftwareBrk(void)
{
    return Software_BRK;
}

void Peripheral_Set_Stop(bool stop)
{
    Stop = stop ? 0x0001U : 0x0000U;
}

bool Peripheral_Get_Stop(void)
{
    return Stop != 0x0000U ? true : false;
}

bool Peripheral_Update_Break(void)
{
    if (Protect_Validate_Flag())
    {
        Stop = 0x0001U;
    }
    if (Stop)
    {
        // 软件触发 BRK
        Software_BRK = true;
        TIMER_SWEVG(TIMER0) |= TIMER_SWEVG_BRKG;
    }
    else
    {
        // Stop = 0，尝试恢复
        if (gpio_input_bit_get(GPIOE, GPIO_PIN_15) == SET)
        {
            Software_BRK = false;
            timer_primary_output_config(TIMER0, ENABLE);  // 恢复 MOE
            Stop = 0;
        }
        else
        {
            Stop = 1;
        }
    }
    return Stop != 0 ? true : false;
}

bool Peripheral_Get_HardwareBrk(void)
{
    return gpio_input_bit_get(GPIOE, GPIO_PIN_15) == SET;
}

void Peripheral_EnableHardwareProtect(void)
{
    timer_interrupt_enable(TIMER0, TIMER_INT_BRK);  // 启用BRK中断
}

void Peripheral_DisableHardwareProtect(void)
{
    timer_interrupt_disable(TIMER0, TIMER_INT_BRK);  // 禁用BRK中断
}

// recommended to operate register if possible //
void Peripheral_Set_PWMChangePoint(Phase_t tcm)
{
    float voltage_bus    = Adc_Get_VoltageBus();
    if(voltage_bus <620.0F)
    {
        Breke_Duty = 1.0F;
    }
    else 
    {
        Breke_Duty = 1-( voltage_bus - 620.0F )/80.0F;
    }
    if (Breke_Duty < 0.0F)
    {
        Breke_Duty = 0.0F;
    }
    if (Breke_Duty > 1.0F)
    {
        Breke_Duty = 1.0F;
    }

    Set_PWM_Compare(tcm.a, tcm.b, tcm.c, Breke_Duty);
}

FloatWithInv_t Peripheral_UpdateUdc(void)
{
    float voltage_bus     = Adc_Get_VoltageBus();
    float voltage_bus_inv = Adc_Get_VoltageBusInv();
    if (Protect_BusVoltage(voltage_bus))
    {
        Stop = 0x0001U;
    }

    FloatWithInv_t result
        = {.val = voltage_bus, .inv = voltage_bus_inv};
    return result;
}

Phase_t Peripheral_Get_PhaseCurrent(void)
{
    Phase_t current_phase;
    Adc_Get_ThreePhaseCurrent(
        &current_phase.a, &current_phase.b, &current_phase.c);
    Protect_PhaseCurrent(current_phase);
    return current_phase;
}

AngleResult_t Peripheral_Update_Position(void)
{
    uint16_t position_data = 0;
    ReadPositionSensor(&position_data);

    Motor_Set_Position(position_data);

    AngleResult_t result;
    result.theta = Motor_Get_ThetaElec();
    result.speed = Motor_Get_Speed();
    return result;
}

void Peripheral_Update_Temperature(void)
{
    float temp = Adc_Get_Temperature();
    if (Protect_Get_FanState(temp))
    {
        gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
    }
    else
    {
        gpio_bit_reset(FAN_OPEN_PORT, FAN_OPEN_PIN);
    }
    if (Protect_Temperature(temp))
    {
        Stop = 0x0001U;
    }
}

void Peripheral_CalibrateADC(void)
{
    Adc_Calibrate_CurrentOffset();
}

void Peripheral_Reset_ProtectFlag(void)
{
    Protect_Reset_Flag();
}

static inline bool can_receive_to_frame(
    const can_receive_message_struct* hw_msg, can_frame_t* frame)
{
    if (!hw_msg || !frame)
    {
        return false;
    }

    // 判断标准帧还是扩展帧
    if (hw_msg->rx_ff == CAN_FF_STANDARD)
    {
        frame->id     = hw_msg->rx_sfid & 0x7FF;  // 11-bit 标准ID
        frame->is_ext = false;
    }
    else
    {
        frame->id     = hw_msg->rx_efid & 0x1FFFFFFF;  // 29-bit 扩展ID
        frame->is_ext = true;
    }

    frame->dlc    = (hw_msg->rx_dlen > 8) ? 8 : hw_msg->rx_dlen;
    frame->is_rtr = (hw_msg->rx_ft == CAN_FT_REMOTE);
    memcpy(frame->data, hw_msg->rx_data, frame->dlc);

    return true;
}
