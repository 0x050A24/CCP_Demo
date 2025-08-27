#include "injection.h"

VoltageInjector_t VoltageInjector = {
    .State = Disable,
    .Count = 0,
    .Vd = 0.0F,
    .Vq = 0.0F,
    .Imax = 0.0F,  // Maximum current for voltage injection
    .PulseWidth = 0
};

void SquareWaveGenerater(VoltageInjector_t* inj, FOC_Parameter_t* foc)
{
  if (inj->State == Enable)
  {
    float ud = 0.0F;
    float uq = 0.0F;

    // Ud 分量判断
    if (inj->Vd >= 0.0F)
    {
      ud = (foc->Id >= inj->Imax) ? -inj->Ud_amp : inj->Ud_amp;
    }
    else
    {
      ud = (foc->Id <= -inj->Imax) ? inj->Ud_amp : -inj->Ud_amp;
    }

    // Uq 分量判断
    if (inj->Vq >= 0.0F)
    {
      uq = (foc->Iq >= inj->Imax) ? -inj->Uq_amp : inj->Uq_amp;
    }
    else
    {
      uq = (foc->Iq <= -inj->Imax) ? inj->Uq_amp : -inj->Uq_amp;
    }

    inj->Vd = ud;
    inj->Vq = uq;
    inj->Count++;
  }
  else
  {
    inj->Vd = 0.0F;
    inj->Vq = 0.0F;
  }
}

void HighFrequencySquareWaveGenerater(VoltageInjector_t* inj)
{
  if (inj->State == Enable)
  {
    if (inj->PulseWidth < 0)
    {
      inj->PulseWidth = 0;
    }

    uint32_t step = inj->Count % (2 * inj->PulseWidth);

    if (step < inj->PulseWidth)
    {
      // 正向脉冲
      inj->Vd = inj->Ud_amp;
      inj->Vq = inj->Uq_amp;
    }
    else
    {
      // 反向脉冲
      inj->Vd = -inj->Ud_amp;
      inj->Vq = -inj->Uq_amp;
    }

    inj->Count++;
  }
  else
  {
    inj->Vd = 0.0f;
    inj->Vq = 0.0f;
    inj->Count = 0;
    inj->Theta = 0.0f;
    inj->PulseWidth = 0;
  }
}
