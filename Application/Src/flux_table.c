#include "flux_table.h"

float lookup_qflux(float iq_input)
{
  // 边界判断：小于最小电流，返回最小磁链
  if (iq_input <= iq_table[0])
  {
    return fluxq_table[0];
  }

  // 边界判断：大于最大电流，返回最大磁链
  if (iq_input >= iq_table[FLUX_TABLE_SIZE - 1])
  {
    return fluxq_table[FLUX_TABLE_SIZE - 1];
  }

  // 查找区间并线性插值
  for (int i = 0; i < FLUX_TABLE_SIZE - 1; i++)
  {
    float iq_low = iq_table[i];
    float iq_high = iq_table[i + 1];

    // 查找到对应区间（包含等于边界的情况）
    if (iq_input >= iq_low && iq_input <= iq_high)
    {
      float fluxq_low = fluxq_table[i];
      float fluxq_high = fluxq_table[i + 1];
      float t = (iq_input - iq_low) / (iq_high - iq_low);
      return fluxq_low + t * (fluxq_high - fluxq_low);
    }
  }

  // 正常情况下永远不会到这里
  return 0.0f;
}

float lookup_dflux(float id_input)
{
  // 边界判断：小于最小电流，返回最小磁链
  if (id_input <= id_table[0])
  {
    return fluxd_table[0];
  }

  // 边界判断：大于最大电流，返回最大磁链
  if (id_input >= id_table[FLUX_TABLE_SIZE - 1])
  {
    return fluxd_table[FLUX_TABLE_SIZE - 1];
  }

  // 查找区间并线性插值
  for (int i = 0; i < FLUX_TABLE_SIZE - 1; i++)
  {
    float id_low = id_table[i];
    float id_high = id_table[i + 1];

    // 查找到对应区间（包含等于边界的情况）
    if (id_input >= id_low && id_input <= id_high)
    {
      float fluxd_low = fluxd_table[i];
      float fluxd_high = fluxd_table[i + 1];
      float t = (id_input - id_low) / (id_high - id_low);
      return fluxd_low + t * (fluxd_high - fluxd_low);
    }
  }

  // 正常情况下永远不会到这里
  return 0.0f;
}
