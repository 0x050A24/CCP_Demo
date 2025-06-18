#ifndef _ADC_H
#define _ADC_H
#include "gd32f30x.h"
#include "systick.h"

extern float Udc;
extern float Ia;
extern float Ib;
extern float Ic;
extern float inv_Udc;

void adc_config_injected(void);
void ADC_Calibration(void);
void ADC_Read_Injection(void);
void ADC_Read_Regular(void);

#endif
