// temp_table.c

#include "temp_table.h"

float adc_to_temp(uint16_t adc)
{
    // 边界保护
    if (adc <= adc_table[0]) return temp_table[0];
    if (adc >= adc_table[TEMP_TABLE_SIZE - 1]) return temp_table[TEMP_TABLE_SIZE - 1];

    // 查找区间并线性插值
    for (int i = 0; i < TEMP_TABLE_SIZE - 1; ++i)
    {
        uint16_t adc_low = adc_table[i];
        uint16_t adc_high = adc_table[i + 1];

        if (adc >= adc_low && adc < adc_high)
        {
            float temp_low = temp_table[i];
            float temp_high = temp_table[i + 1];
            float slope = (temp_high - temp_low) / (adc_high - adc_low);
            return temp_low + slope * (adc - adc_low);
        }
    }

    // 理论不会到达这里
    return 0.0f;
}
