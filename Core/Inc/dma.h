#include "gd32f30x.h"


extern uint32_t adc_value[2];
extern uint32_t usart_txbuffer[2];

void ADC_DMA_Init(void);
void USART_DMA_Init(void);
void USART_DMA_Send(uint32_t data_len);