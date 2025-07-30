#include "injection.h"

#include "common_math.h"

VoltageInjector_t VoltageInjector = {
    .State = Disable,
    .Count = 0,
    .Vd = 0.0F,
    .Vq = 0.0F,
    .Imax = 0.0F  // Maximum current for voltage injection
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

void HighFrequencySquareWaveGenerater(VoltageInjector_t* inj, float Udc)
{
  // 更新注入电压幅值为 √3 * Udc
  inj->Ud_amp = inv_SQRT3 * Udc;
  inj->Uq_amp = 0.0f;  // 如果你只注入 Ud 分量

  if (inj->State == Enable)
  {
    if (inj->PulseWidth <= 0)
    {
      inj->PulseWidth = 1;
    }

    uint32_t step = inj->Count % (4 * inj->PulseWidth);

    if (step < inj->PulseWidth)
    {
      // 正向脉冲
      inj->Vd = inj->Ud_amp;
      inj->Vq = inj->Uq_amp;
    }
    else if (step < 2 * inj->PulseWidth)
    {
      // 空隙（关断）
      inj->Vd = 0.0f;
      inj->Vq = 0.0f;
    }
    else if (step < 3 * inj->PulseWidth)
    {
      // 反向脉冲
      inj->Vd = -inj->Ud_amp;
      inj->Vq = -inj->Uq_amp;
    }
    else
    {
      // 空隙（关断）
      inj->Vd = 0.0f;
      inj->Vq = 0.0f;
    }

    // 每完成一个完整周期（4个脉冲段），更新 Theta
    if (step == (uint32_t)(4 * inj->PulseWidth - 1))
    {
      inj->Theta += M_2PI / 360.0F;
      if (inj->Theta > M_2PI)
      {
        inj->Theta -= M_2PI;
      }
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
