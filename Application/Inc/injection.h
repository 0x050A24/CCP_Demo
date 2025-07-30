#ifndef _INJECTION_H
#define _INJECTION_H

#include "foc_types.h"

typedef enum{
    SQ,
    HF_SQ,
} Wavetype_t;

typedef struct
{
    float Ud_amp;
    float Uq_amp;
    float Vd;
    float Vq;
    float Imax;
    float Theta;
    Wavetype_t Type; // SQ: Square Wave, HF_SQ: High Frequency Square Wave
    EnableStatus State; // 0: Idle, 1: Injecting
    uint32_t Count;      // Counter for injection duration
    uint16_t PulseWidth; // 每个正负极性脉冲宽度（单位：控制周期数）
} VoltageInjector_t;

extern VoltageInjector_t VoltageInjector;

void SquareWaveGenerater(VoltageInjector_t *inj, FOC_Parameter_t *foc);
void HighFrequencySquareWaveGenerater(VoltageInjector_t* inj, float Udc);

#endif
