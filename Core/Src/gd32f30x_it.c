/*!
    \file    gd32f30x_it.c
    \brief   interrupt service routines

   \version 2024-12-20, V3.0.1, firmware for GD32F30x
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f30x_it.h"
#include "main.h"
#include "systick.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1)
    {
    }
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_decrement();
    systick_ms++;
}

void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    can_receive_message_struct rx_msg;

    // 从硬件FIFO取出一帧CAN消息
    can_message_receive(hcan0.Instance, CAN_FIFO0, &rx_msg);

    // 计算下一个head位置，检查缓冲区是否满
    uint8_t next_head = (can_buffer_head + 1) % CAN_BUFFER_SIZE;
    if (next_head != can_buffer_tail)
    { // 有空间
        can_buffer[can_buffer_head] = rx_msg;
        can_buffer_head = next_head;
    }
    else
    {
        // 缓冲区满了，可以统计丢帧数或其他处理
    }
}
extern uint32_t pulse;

void TIMER0_Channel_IRQHandler(void)
{

    if (timer_interrupt_flag_get(TIMER0, TIMER_INT_CH3))
    {
        timer_interrupt_flag_clear(TIMER0, TIMER_INT_CH3);
        adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
    }
}
extern float VF_Freq;
extern float VF_Vref;
void ADC0_1_IRQHandler(void)
{
    if (adc_interrupt_flag_get(ADC0, ADC_INT_FLAG_EOIC))
    {
        adc_interrupt_flag_clear(ADC0, ADC_INT_FLAG_EOIC);
        // uint16_t adc_value_ch0 = (uint16_t)(ADC_IDATA0(ADC0) & 0xFFFF);
        // uint16_t adc_value_ch1 = (uint16_t)(ADC_IDATA1(ADC0) & 0xFFFF);
        // uint16_t adc_value_ch2 = (uint16_t)(ADC_IDATA2(ADC0) & 0xFFFF);
        // uint16_t adc_value_ch3 = (uint16_t)(ADC_IDATA3(ADC0) & 0xFFFF);
        gpio_bit_write(GPIOD, GPIO_PIN_8, 1 - gpio_input_bit_get(GPIOD, GPIO_PIN_8)); // 切换LED状态
        VF_Control(VF_Freq, VF_Vref, 24.0f, 0.0001f, PWM_ARR);                              
    }
}