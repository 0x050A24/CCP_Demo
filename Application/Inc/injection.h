#ifndef _INJECTION_H
#define _INJECTION_H

#include "foc_types.h"

typedef struct
{
  float Ud_amp;
  float Uq_amp;
  float Vd;
  float Vq;
  float Imax;
  float PulseWidth;    // Pulse width for square wave Normalized to [0, 1]
  float Theta;         // Electrical angle, Normalized to [0, 2 * PI]
  EnableStatus State;  // 0: Idle, 1: Injecting
  uint32_t Count;      // Counter for injection duration
} VoltageInjector_t;

extern VoltageInjector_t VoltageInjector;

void SquareWaveGenerater(VoltageInjector_t* inj, FOC_Parameter_t* foc);
void HighFrequencySquareWaveGenerater(VoltageInjector_t* inj);

#endif
