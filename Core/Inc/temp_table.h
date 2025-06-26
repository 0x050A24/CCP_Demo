// temp_table.h

#ifndef _TEMP_TABLE_H_
#define _TEMP_TABLE_H_

#include <stdint.h>

// 插值表大小
#define TEMP_TABLE_SIZE 12

// ADC 输入表（必须升序）
static const uint16_t adc_table[TEMP_TABLE_SIZE] = {
    195, 256, 341, 463, 637, 886, 1232, 1689, 2241, 2823, 3335, 3705
};

// TEMP 输出表
static const float temp_table[TEMP_TABLE_SIZE] = {
    198.0f, 180.0f, 162.0f, 144.0f, 126.0f, 108.0f,
     90.0f,  72.0f,  54.0f,  36.0f,  18.0f,   0.0f
};


float adc_to_temp(uint16_t adc);


#endif // _TEMP_TABLE_H