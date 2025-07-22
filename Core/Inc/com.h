#ifndef COM_H
#define COM_H

#include "com_types.h"
#include "can_frame.h"

void COM_CANProtocolDispatcher(can_rx_message_t* msg, const can_frame_t* frame);
void COM_CANProtocolProcess(can_rx_message_t* msg);

#endif // COM_H
