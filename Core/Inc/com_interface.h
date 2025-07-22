#ifndef _COMM_INTERFACE_H_
#define _COMM_INTERFACE_H_

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"
#include "com_frame.h"

bool Com_ReadCANFrame(can_frame_t* out_frame);
bool Com_CANSendEnqueue(uint32_t id, const uint8_t *data, size_t len);
bool Com_CANSendProcess(void);

bool Com_SCISendEnqueue(float* data, uint8_t floatnum);
bool Com_SCISendProcess(void);

#endif /* _COMM_INTERFACE_H_ */
