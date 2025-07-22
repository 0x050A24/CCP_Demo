#include "com.h"
#include "com_interface.h"
#include "ccp_interface.h"
#include "ccppar.h"
#include "string.h"


static inline bool CAN_to_CCP(const can_frame_t* msg, ccp_message_t* ccp_msg);

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
      ccpCommand(msg->msg.ccp_msg.data);//  Message wrote to Buffer already
      Com_CANTXProcess();
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
