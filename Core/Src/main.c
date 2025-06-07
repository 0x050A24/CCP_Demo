
#include "main.h"

#define DAQ_INTERVAL_CYCLES 600000 // 5ms * 120MHz

float VF_Freq = 0;
float VF_Vref = 0;
float_t float_var = 0.0f;
uint32_t pulse = 0;
uint16_t register_count = 0;

extern GPIO_InitTypeDef GPIOD_InitStruct;
static uint32_t last_daq_ms = 0;

void adc_config_injected(void);
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
    adc_config_injected();
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
    nvic_irq_enable(TIMER0_Channel_IRQn, 1, 0);
    nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 5, 0);
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

void adc_config_injected(void)
{
    /* 1. 启用 ADC 时钟 */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
    rcu_periph_clock_enable(RCU_GPIOA);

    /* 2. 配置 PA0 ~ PA3 为模拟输入 */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    /* 3. 设置为单ADC模式 */
    adc_mode_config(ADC_MODE_FREE);

    /* 4. 扫描模式用于多个注入通道 */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);

    /* 5. 注入通道长度配置为 4 */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 4);

    /* 6. 配置注入通道 */
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_13POINT5); // PA0
    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_1, ADC_SAMPLETIME_13POINT5); // PA1
    adc_inserted_channel_config(ADC0, 2, ADC_CHANNEL_2, ADC_SAMPLETIME_13POINT5); // PA2
    adc_inserted_channel_config(ADC0, 3, ADC_CHANNEL_3, ADC_SAMPLETIME_13POINT5); // PA3

    /* 7. 设置注入转换的触发来源为 Software */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);

    /* 8. 启用外部触发 */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);

    /* 9. 启用 ADC */
    adc_enable(ADC0);
    delay_1ms(1);
    adc_calibration_enable(ADC0);

    nvic_irq_enable(ADC0_1_IRQn, 2, 0);
    adc_interrupt_enable(ADC0, ADC_INT_EOIC);
}
