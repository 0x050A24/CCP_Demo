
#include "main.h"

#define DAQ_INTERVAL_CYCLES 600000 // 5ms * 120MHz

uint16_t test_var1 = 0;
int16_t test_var2 = 0;
float_t float_var = 0.0f;
uint32_t pulse = 0;
uint16_t register_count = 0;
static uint32_t dwt_start_cycle = 0;
extern GPIO_InitTypeDef GPIOD_InitStruct;
static uint32_t last_daq_ms = 0;

void daq_trigger(void);
void nvic_config(void);
void dwt_init(void);
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    dwt_init();
    GPIO_Init(GPIOD, &GPIOD_InitStruct);
    systick_config();
    /* initialize Serial port */
    USART_Init(&husart0);
    /* configure NVIC */
    nvic_config();

    /* initialize Timer */
    TIM0_PWM_Init();

    /* initialize CAN and CCP */
    CAN_Init(&hcan0);
    ccpInit();

    while (1)
    {
        process_can_rx_buffer();
        daq_trigger();
        ccpSendCallBack();
    }
}

/*!
    \brief      configure the nested vectored interrupt controller
    \param[in]  none
    \param[out] none
    \retval     none
*/
void nvic_config(void)
{
    /* configure CAN0 NVIC */
    nvic_irq_enable(TIMER0_Channel_IRQn, 0, 0);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 1, 0);
}

void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能DWT模块
    DWT->CYCCNT = 0;                                // 清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 启用CYCCNT
}

void daq_trigger(void)
{
    if ((systick_ms - last_daq_ms) >= 5)
    {
        last_daq_ms = systick_ms;
        ccpDaq(0);
    }
}