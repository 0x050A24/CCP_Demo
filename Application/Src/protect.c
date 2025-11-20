#include "protect.h"
#include <stdbool.h>
#include <stdint.h>

//static bool           Protect_Enable     = false;
static float          Protect_BusVolRate = 0.0F;
static float          Protect_BusVolFluc = 0.0F;
static float          Protect_CurrentMax = 0.0F;
static float          Protect_TempMax    = 0.0F;
static Protect_Flag_t Flag               = {0};

bool Protect_Initialization(const Protect_Parameter_t* param) {
    if (param == NULL) {
        return false;
    }
    Protect_BusVolRate = param->Udc_rate;
    Protect_BusVolFluc = param->Udc_fluctuation;
    Protect_CurrentMax = param->I_Max;
    Protect_TempMax    = param->Temperature;
    Flag               = param->Flag;
    return true;
}

bool Protect_PhaseCurrent(Phase_t current) {
    float max = Protect_CurrentMax;
    if ((current.a > 0.9 * max || current.a < -0.9 * max)
        || (current.b > 0.9 * max || current.b < -0.9 * max)
        || (current.c > 0.9 * max || current.c < -0.9 * max)) {
        static uint16_t cnt = 0;
        cnt++;
        if (cnt > 10) {
            Flag |= Over_Avg_Current;
            cnt = 0;
        }
    }
    if ((current.a > max || current.a < -1 * max)
        || (current.b > max || current.b < -1 * max)
        || (current.c > max || current.c < -1 * max)) {
        Flag |= Over_Max_Current;
    }
    return Flag & (Over_Avg_Current | Over_Max_Current);
}

bool Protect_BusVoltage(float bus_voltage) {
    if ((bus_voltage > Protect_BusVolRate + Protect_BusVolFluc)) {
        Flag |= Over_Voltage;
    }
    // if ((bus_voltage < Protect_BusVolRate - Protect_BusVolFluc)) {
    if ((bus_voltage < 20.0F)) {
        Flag |= Low_Voltage;
    }
    return Flag & (Over_Voltage | Low_Voltage);
}

void Protect_HardWareFault(bool status) {
    if (!status) {
        Flag |= Hardware_Fault;
    }
}

bool Protect_Temperature(float temperature) {
    if (temperature > Protect_TempMax) {
        Flag |= Over_Heat;
    }
    return Flag & Over_Heat;
}

bool Protect_Get_FanState(float temperature) {
    static bool fan = false;
    if (temperature > 0.40F * Protect_TempMax) {
        fan = true;
    }
    if (temperature < 0.36F * Protect_TempMax) {
        fan = false;
    }
    return fan;
}

bool Protect_Validate_Flag(void) {
    return Flag != No_Protect;
}

void Protect_Reset_Flag(void) {
    Flag = No_Protect;
}
