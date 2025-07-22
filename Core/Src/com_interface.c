#include "com_interface.h"
#include "peripheral_interface.h"
#include "string.h"

#define CAN_TX_BUFFER_SIZE 8

static can_frame_t can_tx_buffer[CAN_TX_BUFFER_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;


bool Com_ReadCANFrame(can_frame_t* out_frame)
{
  return Peripheral_CANRX(out_frame);
}

bool Com_CANTXEnqueue(uint32_t id, const uint8_t* data, size_t len)
{
  if (!data || len > 8) return false;

  uint8_t next_head = (tx_head + 1) % CAN_TX_BUFFER_SIZE;
  if (next_head == tx_tail)
  {
    // 缓冲区满
    return false;
  }

  can_tx_buffer[tx_head].id = id;
  can_tx_buffer[tx_head].dlc = len;
  memcpy(can_tx_buffer[tx_head].data, data, len);
  tx_head = next_head;

  return true;
}

bool Com_CANTXProcess(void)
{
  if (tx_tail != tx_head)
  {
    can_frame_t* msg = &can_tx_buffer[tx_tail];

    if (!Peripheral_CANTX(msg))
    {
      // 邮箱繁忙，暂停发送
      return false;;
    }

    tx_tail = (tx_tail + 1) % CAN_TX_BUFFER_SIZE;
    return true;
  }
  return false;
}

