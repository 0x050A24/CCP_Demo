
#include "main.h"

#define DAQ_INTERVAL_CYCLES 600000 // 5ms * 120MHz

float VF_Freq = 0;
float VF_Vref = 0;
float_t float_var = 0.0f;
uint32_t pulse = 0;
uint16_t register_count = 0;
uint16_t STOP = 1 ;

extern GPIO_InitTypeDef GPIOD_InitStruct;
static uint32_t last_daq_ms = 0;

void adc_config_injected(void);
void daq_trigger(void);
void nvic_config(void);
void pwm_update_state(void);
void relay_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    //DWT initialized at end of SystemInit();
    GPIO_Init(GPIOD, &GPIOD_InitStruct);
    systick_config();
    /* initialize Serial port */
    USART_Init(&husart0);
    /* configure NVIC */
    nvic_config();

    /* initialize Timer */
    TIM0_PWM_Init();
    adc_config_injected();
    /* initialize CAN and CCP */
    CAN_Init(&hcan0);
    ccpInit();

    while (1)
    {
        process_can_rx_buffer();
        daq_trigger();
        ccpSendCallBack();
        pwm_update_state();
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
    nvic_irq_enable(TIMER0_Channel_IRQn, 1, 0);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 5, 0);
}

void daq_trigger(void)
{
    if ((systick_ms - last_daq_ms) >= 5)
    {
        last_daq_ms = systick_ms;
        ccpDaq(0);
    }
}

void relay_init(void)
{
    gpio_bit_set(SOFT_OPEN_PORT, SOFT_OPEN_PIN); 
    gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN); 
}

void pwm_update_state(void)
{
    if (STOP) {
        // 软件触发 BRK
        TIMER_SWEVG(TIMER0) |= TIMER_SWEVG_BRKG;
    } else {
        // STOP = 0，尝试恢复
        //if (gpio_input_bit_get(GPIOE, GPIO_PIN_15) == SET) {
            timer_primary_output_config(TIMER0, ENABLE); // 恢复 MOE
        //}
    }
}
