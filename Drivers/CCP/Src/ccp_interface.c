/*---------------------------------------------------------------------------    */
/* User should implement 6 function:                                             */
/* ccpInit, ccpCommand, ccpDaq, ccpBackground, ccpSendCallback, ccpSend.         */
/* 1.ccpInit: should be called after CAN initialzed                              */
/* 2.ccpCommand: receives CAN message and process CCP message                    */
/* 3.ccpDaq: used if DAQ is needed, should be called at regular time             */
/* 4.ccpBackground: empty for many implementations                               */
/* 5.ccpSendCallback: check and trigger pending message.                         */
/* 6.ccpSend: send CCP message, All messages send form ccpSend                   */
/* ccpCommand, ccpSend, ccpUserBackground should be defined in ccp_interface.c   */
/* ccpInit, ccpDaq, ccpSendCallback should be implemented in the main loop       */
/*---------------------------------------------------------------------------    */
#include "ccp_interface.h"

void ccpUserBackground(void)
{
}

void process_can_rx_buffer(void)
{
    while (can_buffer_tail != can_buffer_head)
    {
        can_receive_message_struct *msg = &can_buffer[can_buffer_tail];
        uint32_t id = (msg->rx_ff == CAN_FF_STANDARD) ? msg->rx_sfid : msg->rx_efid;
        if (id == CCP_CRO_ID && msg->rx_dlen == 8)
        {
            ccpCommand(msg->rx_data);
        }
        can_buffer_tail = (can_buffer_tail + 1) % CAN_BUFFER_SIZE;
    }
}

void ccpSend(CCP_BYTEPTR msg)
{
    can_transmit_message_struct tx_message;

    tx_message.tx_sfid = CCP_DTO_ID;    // 标准帧ID
    tx_message.tx_efid = 0;             // 扩展帧ID，不用
    tx_message.tx_ff = CAN_FF_STANDARD; // 标准帧格式
    tx_message.tx_ft = CAN_FT_DATA;     // 数据帧
    tx_message.tx_dlen = 8;             // 数据长度

    for (int i = 0; i < 8; i++)
    {
        tx_message.tx_data[i] = msg[i];
    }

    uint8_t mailbox = can_message_transmit(hcan0.Instance, &tx_message);
    if (mailbox == CAN_NOMAILBOX)
    {
        // 发送失败处理：无空邮箱
    }
}
