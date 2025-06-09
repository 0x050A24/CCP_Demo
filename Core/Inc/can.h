#ifndef _CAN_H
#define _CAN_H

#define CAN_BUFFER_SIZE 8

#include "gd32f30x.h"
#include "string.h"
#include <stdio.h>

typedef struct {
    uint32_t TX_GPIO_Port;
    uint32_t TX_Pin;
    uint32_t RX_GPIO_Port;
    uint32_t RX_Pin;
    uint32_t BaudrateKbps;
} CAN_InitTypeDef;

typedef struct {
    uint32_t Instance; // CAN0 is the only instance
    CAN_InitTypeDef Init;
} CAN_HandleTypeDef;

typedef enum {
    CAN_OK = 0,
    CAN_ERROR,
} CAN_StatusTypeDef;





CAN_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcan);

extern CAN_HandleTypeDef hcan0;
extern can_transmit_message_struct transmit_message;
extern volatile uint8_t can_received;

extern can_receive_message_struct can_buffer[CAN_BUFFER_SIZE];
extern volatile uint8_t can_buffer_head;
extern volatile uint8_t can_buffer_tail;

#endif /* GD_CAN_H */