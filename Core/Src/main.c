
#include "main.h"

#define DAQ_INTERVAL_CYCLES 600000 // 5ms * 120MHz

volatile uint32_t DWT_Count = 0;

uint16_t pin = 0;


static uint32_t last_daq_ms = 0;

void adc_config_injected(void);
void daq_trigger(void);
void nvic_config(void);
void EXIT_Config(void);

void relay_init(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    // DWT initialized at the end of SystemInit();
    systick_config();
    delay_1ms(100); // wait incase of download
    /* initialize GPIO */
    GPIO_Init(GPIOD, &GPIOD_InitStruct);
    GPIO_Init(GPIOB, &GPIOB_InitStruct);
    /* initialize Serial port */
    USART_Init(&husart0);
    /* configure NVIC */
    nvic_config();
    /* initialize Timer */
    TIM0_PWM_Init();
    /* initialize external interrupt */
    EXIT_Config();
    /* initialize ADC */
    adc_config_injected();
    /* initialize CAN and CCP */
    CAN_Init(&hcan0);
    ccpInit();
    relay_init();

    while (1)
    {
        process_can_rx_buffer();
        daq_trigger();
        ccpSendCallBack();
        Gate_state();
        ADC_Read_Regular();
        pin = gpio_input_bit_get(GPIOE, GPIO_PIN_15);


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
    nvic_irq_enable(TIMER0_BRK_IRQn, 0, 0);
    nvic_irq_enable(EXTI5_9_IRQn, 1U, 0U);
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
    // gpio_bit_set(FAN_OPEN_PORT, FAN_OPEN_PIN);
}

void EXIT_Config(void)
{
    gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOB, GPIO_PIN_SOURCE_7);
    exti_init(EXTI_7, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_flag_clear(EXTI_7);
}
