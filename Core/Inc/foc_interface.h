#ifndef _FOC_INTERFACE_H_
#define _FOC_INTERFACE_H_

#include "foc_types.h"
#include "stdint.h"
#include "dma.h"

extern volatile uint16_t STOP;
extern ControlStatus Software_BRK;
extern Protect_Flags Protect_Flag;

void Interface_UpdateUdc(void);
void Interface_UpdateCurrent(void);
void Interface_UpdatePosition(void);
void Interface_GateState(void);
void Interface_CalibrateADC(void);
void Interface_GetSystemFrequency(void);
void Interface_DMASerialSend(void);
void Interface_EnableHardwareProtect(void);
void Interface_DisableHardwareProtect(void);
void Interface_SetPWMChangePoint(float Tcm1, float Tcm2, float Tcm3, float pwm_arr);

#endif /* _FOC_INTERFACE_H_ */
