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

void HighFrequencySquareWaveGenerater(VoltageInjector_t* inj)
{
  uint16_t step = inj->Count % 10;

  if (inj->State == Enable)
  {
    if (step == 0 || step == 1)
    {
      // 注入正向电压（使用当前 Theta，不更新）
      inj->Vd = inj->Ud_amp;
      inj->Vq = inj->Uq_amp;
    }
    else if (step == 2 || step == 3)
    {
      // 注入反向电压（使用当前 Theta，不更新）
      inj->Vd = -inj->Ud_amp;
      inj->Vq = -inj->Uq_amp;
    }
    else
    {
      inj->Vd = 0.0F;
      inj->Vq = 0.0F;
    }

    // 注入周期结束后（9 -> 0），更新 Theta（延后更新）
    if (step == 9)
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
    // 非注入状态保持零电压
    inj->Vd = 0.0F;
    inj->Vq = 0.0F;
    inj->Count = 0;
    inj->Theta = 0.0F;  // 重置 Theta
  }
}
