#include "com_interface.h"
#include "peripheral_interface.h"
#include "string.h"

#define CAN_TX_BUFFER_SIZE 8

#define SCI_TX_BUFFER_SIZE 8
#define MAX_FLOATS_PER_FRAME 8

static can_frame_t can_tx_buffer[CAN_TX_BUFFER_SIZE];
static volatile uint8_t can_tx_head = 0;
static volatile uint8_t can_tx_tail = 0;

static float sci_tx_buffer[SCI_TX_BUFFER_SIZE][MAX_FLOATS_PER_FRAME];
static uint8_t sci_tx_lengths[SCI_TX_BUFFER_SIZE]; // 每帧浮点数量
static volatile uint8_t sci_tx_head = 0;
static volatile uint8_t sci_tx_tail = 0;



bool Com_ReadCANFrame(can_frame_t* out_frame)
{
  return Peripheral_CANReceive(out_frame);
}

bool Com_CANSendEnqueue(uint32_t id, const uint8_t* data, size_t len)
{
  if (!data || len > 8) return false;

  uint8_t new_head = (can_tx_head + 1) % CAN_TX_BUFFER_SIZE;

  // 缓冲区满，丢弃“旧数据”：直接前移 tail
  if (new_head == can_tx_tail)
  {
    can_tx_tail = (can_tx_tail + 1) % CAN_TX_BUFFER_SIZE;  // 丢掉最旧的
  }

  // 写入新数据
  can_tx_head = new_head;
  can_tx_buffer[can_tx_head].id = id;
  can_tx_buffer[can_tx_head].dlc = len;
  memcpy(can_tx_buffer[can_tx_head].data, data, len);

  return true;
}

bool Com_CANSendProcess(void)
{
  if (can_tx_head != can_tx_tail)
  {
    // 取出最新的数据（head指向最新）
    can_frame_t* msg = &can_tx_buffer[can_tx_head];

    if (!Peripheral_CANSend(msg))
    {
      // CAN邮箱繁忙，暂停
      return false;
    }

    // 发送完后，将 head 往“旧的方向”移动
    if (can_tx_head == 0) {
      can_tx_head = CAN_TX_BUFFER_SIZE - 1;
    } else {
      can_tx_head--;
}

    return true;
  }

  return false;
}

bool Com_SCISendEnqueue(float* data, uint8_t float_count)
{
    uint8_t next_head = (sci_tx_head + 1) % SCI_TX_BUFFER_SIZE;
    if (next_head == sci_tx_tail) {
        return false; // 缓冲区满
    }

    memcpy(sci_tx_buffer[sci_tx_head], data, float_count * sizeof(float));
    sci_tx_lengths[sci_tx_head] = float_count;
    sci_tx_head = next_head;
    return true;
}

bool Com_SCISendProcess(void)
{
    if (sci_tx_tail == sci_tx_head) return false;

    float* data = sci_tx_buffer[sci_tx_tail];
    uint8_t count = sci_tx_lengths[sci_tx_tail];

    Peripheral_SCISend(data, count);
    sci_tx_tail = (sci_tx_tail + 1) % SCI_TX_BUFFER_SIZE;
    return true;
}


