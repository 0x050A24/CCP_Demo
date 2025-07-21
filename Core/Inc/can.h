#ifndef _CAN_H
#define _CAN_H

#define CAN_BUFFER_SIZE 8

#include "gd32f30x_can.h"
#include "stdbool.h"

bool CAN_Init(void);
bool CAN_Buffer_Put(const can_receive_message_struct* msg);
bool CAN_Buffer_Get(can_receive_message_struct *msg);

#endif /* GD_CAN_H */
