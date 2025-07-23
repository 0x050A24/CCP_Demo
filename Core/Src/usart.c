#include "usart.h"

#define DATA_SIZE 16 // used for Serial DMA buffer, must be big enough to possible transmit data

extern uint32_t adc_value[2];

USART_HandleTypeDef husart0 = {
    .Instance = USART0,          // USART 外设
    .BaudRate = USART_BAUDRATE,         // 波特率
    .WordLength = USART_WL_8BIT, // 数据位：8位
    .StopBits = USART_STB_1BIT,  // 停止位：1位
    .Parity = USART_PM_NONE,     // 校验位：无校验
    .TxGPIO_Port = GPIOA,        // TX 端口
    .RxGPIO_Port = GPIOA,        // RX 端口
    .TxPin = GPIO_PIN_9,         // TX 引脚
    .RxPin = GPIO_PIN_10,        // RX 引脚
    .USART_CLK = RCU_USART0,     // USART 时钟
    .GPIO_CLK = RCU_GPIOA        // GPIO 时钟
};

union
{
    float f;
    uint32_t u;
} conv;

void USART_Init(USART_HandleTypeDef *husart)
{
    /* 使能 GPIO 和 USART 时钟 */
    rcu_periph_clock_enable(husart->GPIO_CLK);
    rcu_periph_clock_enable(husart->USART_CLK);

    /* 配置 USART Tx 引脚为复用推挽模式 */
    gpio_init(husart->TxGPIO_Port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, husart->TxPin);

    /* 配置 USART Rx 引脚为浮空输入 */
    gpio_init(husart->RxGPIO_Port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, husart->RxPin);

    /* USART 配置 */
    usart_deinit(husart->Instance);
    usart_baudrate_set(husart->Instance, husart->BaudRate);
    usart_receive_config(husart->Instance, USART_RECEIVE_ENABLE);
    usart_transmit_config(husart->Instance, USART_TRANSMIT_ENABLE);
    usart_word_length_set(husart->Instance, husart->WordLength); // 数据位：8位
    usart_stop_bit_set(husart->Instance, husart->StopBits);      // 停止位：1位
    usart_parity_config(husart->Instance, husart->Parity);       // 校验位：无校验

    usart_enable(husart->Instance);
}



void USART_DMA_Init(void)
{
    /*DMA初始化*/
    dma_parameter_struct dma_init_struct;
    // 时钟开启
    rcu_periph_clock_enable(RCU_DMA0);
    dma_deinit(DMA0, DMA_CH3);                                     // dma寄存器初始化
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;          // 传输模式，存储到外设（发送）
    dma_init_struct.memory_addr = 0x0;                             // dma内存地址
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;       // 内存地址增量模式
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;          // read 8bits once
    dma_init_struct.number = 0;                                    // 长度
    dma_init_struct.periph_addr = (uint32_t)(&USART_DATA(USART0)); // 外设基地址( (uint32_t)USART_DATA(USART0) )
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;      // 外设地址增量禁用
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_HIGH; // 优先级高
    dma_init(DMA0, DMA_CH3, &dma_init_struct);

    /* configure DMA mode */
    
    
    dma_circulation_disable(DMA0, DMA_CH3);                       // 循环模式禁用
    dma_memory_to_memory_disable(DMA0, DMA_CH3);                  // 通道3   USART0_TX
    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE); // USART0 DMA发送使能
    dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
}

//< For VOFA+ justfloat frame, this is the most stable way to send data >//
void USART_DMA_Send(float* TxBuffer, uint8_t floatnum)
{
    dma_channel_disable(DMA0, DMA_CH3);

    dma_memory_address_config(DMA0, DMA_CH3, (uint32_t)TxBuffer);

    dma_transfer_number_config(DMA0, DMA_CH3, 4 * floatnum);

    dma_channel_enable(DMA0, DMA_CH3);
}

// This function provides printf redirection
// NOLINTNEXTLINE(bugprone-reserved-identifier)
int _write(int file, char *data, int len)
{
    if ((file == STDOUT_FILENO) || (file == STDERR_FILENO))
    {
        for (int i = 0; i < len; i++)
        {
            usart_data_transmit(husart0.Instance, (uint8_t)data[i]);
            while (RESET == usart_flag_get(husart0.Instance, USART_FLAG_TBE))
            {
                ;
            }
        }
        return len;
    }
    return -1; // 如果不是标准输出，返回错误
}
