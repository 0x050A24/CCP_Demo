#ifndef _FOC_INTERFACE_H_
#define _FOC_INTERFACE_H_

#include "stdint.h"
#include "foc.h" // IWYU pragma: export, use foc as a package
#include "stdbool.h"
#include "ccp_interface.h" // IWYU pragma: export, use ccp as a package


#define Temperature_Threshold 80.0F
#define Current_Threshold 10.0F
#define Voltage_Rate 220.0F
#define Voltage_Fluctuation 40.0F

typedef enum
{
    No_Protect = 0,
    Over_Current = 1 << 0,          // 0b0001
    Over_Maximum_Current = 1 << 1,  // 0b0010
    Over_Voltage = 1 << 2,          // 0b0100
    Low_Voltage = 1 << 3,           // 0b1000
    Hardware_Fault = 1 << 4,        // 0b10000
    Over_Heat = 1 << 5              // 0b100000
} Protect_Flags;

typedef struct
{
  float Udc_rate;          // Udc rate
  float Udc_fluctuation;   // Udc fluctuation
  float I_Max;
  float Temperature;
  Protect_Flags Flag;
} Protect_Parameter_t;

typedef enum {
    NONE = 0,
    CCP,
    CUSTOM, // Reserved for future use
} ProtocolType_t;

typedef struct {
    ProtocolType_t Protocol;
    union {
        ccp_message_t ccp_msg;
    } msg;
} can_rx_message_t;


extern volatile uint16_t STOP;
extern EnableStatus Software_BRK;
extern Protect_Parameter_t Protect;

bool Interface_ReadCANMessage(can_rx_message_t* out_msg);
bool Interface_CANTXEnqueue(uint32_t id, const uint8_t *data, size_t len);
void Interface_CANTXProcess(void);
void Interface_InitProtectParameter(void);
void Interface_UpdateUdc(void);
void Interface_UpdateCurrent(void);
void Interface_UpdatePosition(void);
void Interface_GateState(void);
void Interface_CalibrateADC(void);
void Interface_GetSystemFrequency(void);
void Interface_DMASerialSend(float* TxBuffer,uint16_t DataSize);
void Interface_EnableHardwareProtect(void);
void Interface_DisableHardwareProtect(void);
void Interface_SetPWMChangePoint(void);

#endif /* _FOC_INTERFACE_H_ */
