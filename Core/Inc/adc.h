#ifndef _ADC_H
#define _ADC_H

// Udc, inverse Udc, Temperature is readed in main loop
extern float Temperature;

void ADC_Init(void);
void ADC_DMA_Init(void);
void ADC_Calibration(void);
void ADC_Read_Injection(float *Ia, float *Ib, float *Ic);
void ADC_Read_Regular(float *Udc, float *inv_Udc);

#endif
