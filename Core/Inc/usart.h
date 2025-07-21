#ifndef _USART_H
#define _USART_H

#include "gd32f30x.h" // IWYU pragma: export
#include <unistd.h>

#define USART_BAUDRATE 500000U

/* 定义一个 USART 句柄结构体 */
typedef struct
{
    uint32_t Instance;         // USART外设基地址，如 USART0 或 USART1
    uint32_t BaudRate;         // 波特率
    uint32_t WordLength;       // 数据位，比如 USART_WL_8BIT
    uint32_t StopBits;         // 停止位，比如 USART_STB_1BIT
    uint32_t Parity;           // 校验位，比如 USART_PM_NONE
    uint32_t TxGPIO_Port;      // 发送引脚所在端口
    uint32_t RxGPIO_Port;      // 接收引脚所在端口
    uint32_t TxPin;            // 发送引脚号
    uint32_t RxPin;            // 接收引脚号
    rcu_periph_enum USART_CLK; // USART时钟
    rcu_periph_enum GPIO_CLK;  // GPIO时钟
} USART_HandleTypeDef;

extern USART_HandleTypeDef husart0; // 声明一个外部的 USART 句柄

/* USART 初始化函数声明 */
void USART_Init(USART_HandleTypeDef *husart);
void USART_DMA_Init(void);
void USART_DMA_Send_Vofa(float* TxBuffer, uint16_t DataSize);

#endif /* GD_USART_H */
