#include "com.h"
#include "ccp_interface.h"
#include "ccppar.h"
#include "com_interface.h"
#include "string.h"
#include "com_types.h"
#include "com_frame.h"

static inline bool CAN_to_CCP(const can_frame_t* msg, ccp_message_t* ccp_msg);
static inline void COM_CANProtocolDispatcher(can_rx_message_t* msg, const can_frame_t* frame);
static inline void COM_CANProtocolProcess(can_rx_message_t* msg);

void COM_CANProtocol(void)
{
  can_rx_message_t can_msg;
  can_frame_t can_frame;
  if (Com_ReadCANFrame(&can_frame))
  {
    COM_CANProtocolDispatcher(&can_msg, &can_frame);
    COM_CANProtocolProcess(&can_msg);
  }
}

void COM_SCIProtocol(void)
{
  
  Com_SCISendProcess();
}

void COM_DAQProtocol(uint32_t systick_ms)
{
  static uint32_t last_daq_ms = 0;

  if ((systick_ms - last_daq_ms) >= 5)
  {
    last_daq_ms = systick_ms;
    ccpDaq(0);
  }

}

void COM_CANProtocolDispatcher(can_rx_message_t* msg, const can_frame_t* frame)
{
  if (CAN_to_CCP(frame, &msg->msg.ccp_msg))
  {
    msg->Protocol = CCP;
    ccp_message_t* ccp_msg = &msg->msg.ccp_msg;
    if (ccp_msg)
    {
      memcpy(ccp_msg->data, frame->data, frame->dlc);
    }
  }
  else
  {
    msg->Protocol = NONE;  // Designed 1 CAN Protocol, extend if needed
  }
}

void COM_CANProtocolProcess(can_rx_message_t* msg)
{
  switch (msg->Protocol)
  {
    case CCP:
      ccpCommand(msg->msg.ccp_msg.data);  //  Message wrote to Buffer already
      Com_CANSendProcess();
      ccpSendCallBack();

      break;
    case NONE:
    default:
      break;
  }
}

static inline bool CAN_to_CCP(const can_frame_t* frame, ccp_message_t* ccp_msg)
{
  if (!frame || !ccp_msg)
  {
    return false;
  }

  uint32_t id = frame->id;
  if (id != CCP_CRO_ID || frame->dlc != 8)
  {
    return false;
  }

  memcpy(ccp_msg->data, frame->data, 8);
  return true;
}
