#ifndef _POSITION_SENSOR_H_
#define _POSITION_SENSOR_H_

#include "gd32f30x.h"
#include "gpio.h"
#include "systick.h"
#include "delay.h"

/*  Gate polarity definition */
#ifndef Encoder_Position
#ifndef Resolver_Position
#define Encoder_Position /* Define here */
#endif
#endif

#if !defined(Encoder_Position) && !defined(Resolver_Position) && !defined(Software_Position)
#error "Please define Positon Sensor Entry in positon_sensor.h"
#endif

#define Excitation_Frequency 0x28  // 10kHz
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

extern uint16_t Position_Data;
extern uint8_t Resolver_Fault;
extern ErrStatus AD2S1210_Config;
extern ErrStatus AD2S1210_Ready;

void Position_Sensor_Init(void);
void ReadPosition(void);

#endif // _POSITION_SENSOR_H_