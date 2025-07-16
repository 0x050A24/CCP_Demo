#include "injection.h"

VoltageInjector_t VoltageInjector = {
    .State = DISABLE,
    .Count = 0,
    .Vd = 0.0f,
    .Vq = 0.0f,
    .Imax = 5.0f // Maximum current for voltage injection
};

void SquareWaveGenerater(VoltageInjector_t *inj, Park_t *park)
{
    if (inj->State == ENABLE)
    {
        float ud = 0.0f;
        float uq = 0.0f;

        if (park->Id >= inj->Imax && inj->Vd >= 0.0f)
        {
            ud = -inj->Ud_amp;
        }
        else if (park->Id <= -inj->Imax && inj->Vd < 0.0f)
        {
            ud = inj->Ud_amp;
        }
        else if (park->Id < inj->Imax && inj->Vd >= 0.0f)
        {
            ud = inj->Ud_amp;
        }
        else if (park->Id > -inj->Imax && inj->Vd < 0.0f)
        {
            ud = -inj->Ud_amp;
        }
        inj->Vd = ud;
        inj->Vq = uq;
        inj->Count++;
    }
    else
    {
        inj->Vd = 0.0f;
        inj->Vq = 0.0f;
        inj->Count = 0;
    }
}