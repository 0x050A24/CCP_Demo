#include "dma.h"

uint32_t adc_value[2];
uint32_t usart_txbuffer[2];

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

void USART_DMA_Init(void)
{
/*DMA初始化*/
    dma_parameter_struct dma_init_struct;
    // 时钟开启
    rcu_periph_clock_enable(RCU_DMA0);
    dma_deinit(DMA0, DMA_CH3);                                     // dma寄存器初始化
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;          // 传输模式，存储到外设（发送）
    dma_init_struct.memory_addr = 0x0;       // dma内存地址
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;       // 内存地址增量模式
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;          // dma外设宽度8位
    dma_init_struct.number = 0;        // 长度
    dma_init_struct.periph_addr = (uint32_t)(&USART_DATA(USART0)); // 外设基地址( (uint32_t)USART_DATA(USART0) )
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;      // 外设地址增量禁用
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_HIGH; // 优先级高
    dma_init(DMA0, DMA_CH3, &dma_init_struct);

    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH3);      // 循环模式禁用
    dma_memory_to_memory_disable(DMA0, DMA_CH3); // 通道3   USART0_TX
    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE); // USART0 DMA发送使能

}
void USART_DMA_Send(uint32_t data_len)

{

        dma_channel_disable(DMA0, DMA_CH3);

        dma_memory_address_config(DMA0, DMA_CH3,(uint32_t)&usart_txbuffer);

        dma_transfer_number_config(DMA0, DMA_CH3, data_len);

        dma_channel_enable(DMA0, DMA_CH3);
}