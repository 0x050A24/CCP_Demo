#include "stdint.h"

#define FLOAT_NUM 3 // number of float to send without frame tail
#define DATA_SIZE (FLOAT_NUM + 1) // +1 for frame tail

extern uint32_t adc_value[2];
extern float usart_txbuffer[DATA_SIZE];

void ADC_DMA_Init(void);
void USART_DMA_Init(void);
void USART_DMA_Send_Vofa(void);
