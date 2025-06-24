
#include "main.h"

volatile uint32_t DWT_Count = 0;
volatile uint32_t TIMER1_Count = 0;

uint16_t pin = 0;
uint16_t receive = 0;
uint16_t transmit = 0;

void DWT_Init(void);
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
    systick_config();
    TIM1_Init();
    //DWT_Init(); // delay_us based on DWT
    /* initialize Serial port */
    USART_Init(&husart0);
    /* initialize GPIO */
    GPIO_Init(GPIOD, &GPIOD_InitStruct);
    GPIO_Init(GPIOB, &GPIOB_InitStruct);
    GPIO_Init(GPIOE, &GPIOE_InitStruct);

    /* initialize AD2S1210 */
    SPI_Init();
    AD2S1210_Init();
    /* initialize Timer */
    TIM0_PWM_Init();
    
    /* initialize external interrupt */
    EXIT_Config();
    /* initialize ADC */
    adc_config_injected();
    /* initialize CAN and CCP */
    CAN_Init(&hcan0);
    ccpInit();
    /* open fan and relay */
    relay_init();
    /* configure NVIC and enable interrupt */
    nvic_config();
    while (1)
    {
        process_can_rx_buffer();
        daq_trigger();
        ccpSendCallBack();
        Gate_state();
        ADC_Read_Regular();
        pin = gpio_input_bit_get(GPIOE, GPIO_PIN_7);

        //DWT_Count = DWT->CYCCNT; // 读取DWT计数器
        TIMER1_Count = TIMER_CNT(TIMER1);
        


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
    nvic_irq_enable(ADC0_1_IRQn, 2, 0);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 5, 0);
    /* SysTick_IRQn 009U */
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
}

void daq_trigger(void)
{
    static uint32_t last_daq_ms = 0;
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

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能DWT模块
    DWT->CYCCNT = 0;                                // 清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 启用CYCCNT
}
