#ifndef _COMM_INTERFACE_H_
#define _COMM_INTERFACE_H_

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"
#include "can_frame.h"


bool Com_CANTXEnqueue(uint32_t id, const uint8_t *data, size_t len);
bool Com_CANTXProcess(void);
bool Com_ReadCANFrame(can_frame_t* out_frame);

#endif /* _COMM_INTERFACE_H_ */
