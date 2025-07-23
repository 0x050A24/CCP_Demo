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
#include "ccppar.h"
#include "stddef.h"

void ccpInit(void);

/* DAQ processor */
void ccpDaq(CCP_BYTE eventChannel);

/* Command processor */
void ccpCommand(CCP_BYTEPTR msg);

/* Transmit Notification */
/* Returns 0 when the CCP driver is idle */
CCP_BYTE ccpSendCallBack(void);

void ccpUserBackground(void);  // empty for many implementations
#endif
