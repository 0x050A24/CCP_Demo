#include "adc.h"
#include "gd32f30x.h"
#include "systick.h"
#include "temp_table.h"

float Temperature = 0.0f;

static uint32_t dc_raw = 0; //may need to be modified though CCP
static uint32_t adc_value[2]; // Regular ADC Channel Buffer
static float adc_ch0_offset = 0;
static float adc_ch1_offset = 0;
static float adc_ch2_offset = 0;

void ADC_Calibration(void)
{
    for (uint16_t offset_count = 0; offset_count < 2000; offset_count++)
    {
        float adc_value_ch0 = (float)(ADC_IDATA0(ADC0) & 0xFFFF);
        float adc_value_ch1 = (float)(ADC_IDATA1(ADC0) & 0xFFFF);
        float adc_value_ch2 = (float)(ADC_IDATA2(ADC0) & 0xFFFF);
        adc_ch0_offset = 0.05f * adc_value_ch0 + 0.95f * adc_ch0_offset;
        adc_ch1_offset = 0.05f * adc_value_ch1 + 0.95f * adc_ch1_offset;
        adc_ch2_offset = 0.05f * adc_value_ch2 + 0.95f * adc_ch2_offset;
    }
}

void ADC_Read_Injection(float* Ia, float* Ib, float* Ic)
{
    // 读取注入通道数据
    float adc_value_ch0 = (float)(ADC_IDATA0(ADC0) & 0xFFFF);
    float adc_value_ch1 = (float)(ADC_IDATA1(ADC0) & 0xFFFF);
    float adc_value_ch2 = (float)(ADC_IDATA2(ADC0) & 0xFFFF);

    // 计算实际电流值
    //> 电流正方向为流出控制器，流入电机 <//
    //< ( 8*1.1*Inom<17> / sqrt(2)*4095 ) = 0.025832277036754 >//
    *Ia = 0.025832277036754f * (adc_value_ch0 - adc_ch0_offset);
    *Ib = 0.025832277036754f * (adc_value_ch1 - adc_ch1_offset);
    *Ic = 0.025832277036754f * (adc_value_ch2 - adc_ch2_offset);
}

void ADC_Read_Regular(float* Udc, float* inv_Udc)
{
    // //< 916 for 224V 58 for 0V >//
    dc_raw = adc_value[0] & 0xFFFF;
    *Udc = 0.2686202686202686f * ((float)dc_raw - 88.0f);
    *inv_Udc = (1.0f / *Udc) > 0.01f ? 0.0f : (1.0f / *Udc);
    Temperature = adc_to_temp(adc_value[1] & 0xFFFF);
}

void ADC_Init(void)
{
    /* 启用 ADC 时钟 */
    rcu_periph_clock_enable(RCU_ADC0);
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOC);

    /* 配置 PA0 ~ PA3 PC5 为模拟输入 */
    gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_MAX,
              GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_5);

    /* 设置为单ADC模式 */
    adc_mode_config(ADC_MODE_FREE);

    /* 右对齐 */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);

    /* 扫描模式用于注入通道 规则通道额外使用连续模式 */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);

    /* 配置注入通道 */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 3);
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_13POINT5);  // PA0
    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_1, ADC_SAMPLETIME_13POINT5);  // PA1
    adc_inserted_channel_config(ADC0, 2, ADC_CHANNEL_2, ADC_SAMPLETIME_13POINT5);  // PA2

    /* 设置注入转换的触发来源为 Timer1 Trigger Out Event: Timer1 Update Event */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_T0_TRGO);

    /* 配置规则通道 */
    adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 2);
    adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_3, ADC_SAMPLETIME_13POINT5);   // PA3 Udc
    adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_15, ADC_SAMPLETIME_13POINT5);  // PC5 NTC

    /* 设置规则通道转换的触发来源为 Software */
    adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);

    /* 启用外部触发 */
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);

    /* 启用 ADC */
    adc_enable(ADC0);
    delay_ms(1);
    adc_calibration_enable(ADC0);

    /* 开启DMA */
    adc_dma_mode_enable(ADC0);

    /* 触发规则通道 */
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);  // 触发一次即开始连续采样
}

void ADC_DMA_Init(void)
{
    /* ADC_DMA_channel configuration */
    dma_parameter_struct dma_data_parameter;

    rcu_periph_clock_enable(RCU_DMA0);

    /* ADC_DMA_channel deinit */
    dma_deinit(DMA0, DMA_CH0);

    /* initialize DMA single data mode */
    dma_data_parameter.periph_addr = (uint32_t)(&ADC_RDATA(ADC0));
    dma_data_parameter.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_data_parameter.memory_addr = (uint32_t)(adc_value);
    dma_data_parameter.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_32BIT;
    dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_32BIT;
    dma_data_parameter.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_data_parameter.number = 2;
    dma_data_parameter.priority = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH0, &dma_data_parameter);

    dma_circulation_enable(DMA0, DMA_CH0);

    /* enable DMA channel */
    dma_channel_enable(DMA0, DMA_CH0);
}
