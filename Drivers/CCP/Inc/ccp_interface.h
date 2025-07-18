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
#ifndef __CCP_INTERFACE_H_
#define __CCP_INTERFACE_H_
//< this is embrella header >//
#include "ccp.h" // IWYU pragma: export
#include "can.h" // IWYU pragma: export

extern void ccpSend(CCP_BYTEPTR msg);
void process_can_rx_buffer(void);
void ccpUserBackground(void);
#endif
