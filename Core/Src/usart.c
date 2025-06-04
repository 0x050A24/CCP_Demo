#include "usart.h"

USART_HandleTypeDef husart0 = {
    .Instance = USART0,          // USART 外设
    .BaudRate = 115200U,         // 波特率
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
