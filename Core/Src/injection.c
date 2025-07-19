#include "injection.h"

VoltageInjector_t VoltageInjector = {
    .State = DISABLE,
    .Count = 0,
    .Vd = 0.0F,
    .Vq = 0.0F,
    .Imax = 0.0F  // Maximum current for voltage injection
};

void SquareWaveGenerater(VoltageInjector_t* inj, FOC_Parameter_t* foc) {
    if (inj->State == ENABLE) {
        float ud = 0.0F;
        float uq = 0.0F;

        // Ud 分量判断
        if (inj->Vd >= 0.0F) {
            ud = (foc->Id >= inj->Imax) ? -inj->Ud_amp : inj->Ud_amp;
        } else {
            ud = (foc->Id <= -inj->Imax) ? inj->Ud_amp : -inj->Ud_amp;
        }

        // Uq 分量判断
        if (inj->Vq >= 0.0F) {
            uq = (foc->Iq >= inj->Imax) ? -inj->Uq_amp : inj->Uq_amp;
        } else {
            uq = (foc->Iq <= -inj->Imax) ? inj->Uq_amp : -inj->Uq_amp;
        }

        inj->Vd = ud;
        inj->Vq = uq;
        inj->Count++;
    } else {
        inj->Vd = 0.0F;
        inj->Vq = 0.0F;
    }
}
