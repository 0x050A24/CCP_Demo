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
#include "com_interface.h"
#include "ccppar.h"

void ccpSend(CCP_BYTEPTR msg);

void ccpUserBackground(void)
{
}

// ccpSend will be called by ccp.c
void ccpSend(CCP_BYTEPTR msg)
{
    uint32_t id = CCP_DTO_ID;   // MCU to PC ID
    size_t len = 8;             // CCP发送固定8字节

    Com_CANSendEnqueue(id, msg, len);
}
