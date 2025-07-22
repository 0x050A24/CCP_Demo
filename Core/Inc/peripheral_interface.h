#ifndef _PHERIPHERAL_INTERFACE_H_
#define _PHERIPHERAL_INTERFACE_H_

#include "stdint.h"
#include "foc.h" // IWYU pragma: export, use foc as a package
#include "stdbool.h"
#include "can_frame.h"



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




extern volatile uint16_t STOP;
extern bool Software_BRK;
extern Protect_Parameter_t Protect;



bool Peripheral_CANTX(const can_frame_t* frame);
bool Peripheral_CANRX(can_frame_t* frame);
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

#endif /* _PHERIPHERAL_INTERFACE_H_ */
