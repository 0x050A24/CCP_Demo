#ifndef _PHERIPHERAL_INTERFACE_H_
#define _PHERIPHERAL_INTERFACE_H_

#include "com_frame.h"
#include "foc.h"  // IWYU pragma: export, use foc as a package
#include "stdbool.h"
#include "stdint.h"

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
  float Udc_rate;         // Udc rate
  float Udc_fluctuation;  // Udc fluctuation
  float I_Max;
  float Temperature;
  Protect_Flags Flag;
} Protect_Parameter_t;

extern volatile uint16_t STOP;
extern bool Software_BRK;
extern Protect_Parameter_t Protect;


void Peripheral_GateState(void);

void Peripheral_InitProtectParameter(void);

bool Peripheral_CANSend(const can_frame_t* frame);
bool Peripheral_CANReceive(can_frame_t* frame);
void Peripheral_SCISend(float* TxBuffer, uint8_t floatnum);
void Peripheral_SCISendCallback(void);

void Peripheral_CalibrateADC(void);
void Peripheral_UpdateUdc(void);
void Peripheral_UpdateCurrent(void);
void Peripheral_UpdatePosition(void);
void Peripheral_GetSystemFrequency(void);

void Peripheral_TemperatureProtect(void);
void Peripheral_EnableHardwareProtect(void);
void Peripheral_DisableHardwareProtect(void);

void Peripheral_SetPWMChangePoint(void);

#endif /* _PHERIPHERAL_INTERFACE_H_ */
