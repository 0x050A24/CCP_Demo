#ifndef _SPI_H_
#define _SPI_H_

#include "gd32f30x.h"
#include "gpio.h"
#include "systick.h"

#define Excitation_Frequency 0x28 // 10kHz
#define Control_Register_Data 0x7D // 14 bits Resolution, 16 bits EncoderResolution

#define A0PORT GPIOA
#define A0Pin GPIO_PIN_6
#define A1PORT GPIOA
#define A1Pin GPIO_PIN_7
#define RESETPORT GPIOB
#define RESETPin GPIO_PIN_6
#define WRPORT GPIOD
#define WRPin GPIO_PIN_11
#define SAMPLEPORT GPIOD
#define SAMPLEPin GPIO_PIN_10

#define LOS_REG 0x88
#define EXCITE_REG 0X91
#define CONTROL_REG 0x92
#define ERROR_REG 0xFF



void SPI_Init(void);
void AD2S1210_Init(void);
uint16_t AD2S1210_Read(void);
uint8_t spi_send_receive_byte(uint32_t spi_periph, uint8_t byte);

#endif
