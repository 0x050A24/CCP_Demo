// 滤波器头文件
#ifndef __FILTER_H__
#define __FILTER_H__

#include <stdbool.h>
#include <stddef.h>
#include "parameters.h" /* CMSIS-DSP math */  // IWYU pragma: export

#define MAX_FILTER_SIZE 32

#ifndef SQRT2
#    define SQRT2 1.414213562373095F
#endif

#ifndef SQRT3
#    define SQRT3 1.73205080757F
#endif

#ifndef SQRT3_2
#    define SQRT3_2 0.86602540378F /* √3/2 */
#endif

#ifndef M_PI
#    define M_PI 3.14159265358979323846F /* π */
#endif

#ifndef M_2PI
#    define M_2PI 6.28318530717958647692F /* 2π */
#endif

#ifndef M_PI_2
#    define M_PI_2 1.57079632679489661923F /* π/2 */
#endif

/*  DSP math function    */
#ifndef ARM_DSP
#    define ARM_DSP
#endif

#ifdef ARM_DSP
#    include "arm_math.h" /* CMSIS-DSP math */  // IWYU pragma: export

#    define COS(x) arm_cos_f32(x)
#    define SIN(x) arm_sin_f32(x)

#else
#    include <math.h>

#    define COS(x) cosf(x)
#    define SIN(x) sinf(x)

#endif

/**
 * @brief Low pass filter structure
 */
typedef struct
{
    float alpha;        // Filter coefficient
    float prev_output;  // Previous output value
    bool  initialized;  // Initialization flag
} IIR1stFilter_t;

/**
 * @brief Band pass filter structure (using two cascaded
 * filters)
 */
typedef struct
{
    // 滤波器系数
    float b0, b1, b2;  // 分子系数
    float a1, a2;      // 分母系数（a0固定为1）

    // 延迟线（存储过去的输入和输出）
    float x1, x2;       // 过去的输入
    float y1, y2;       // 过去的输出
    bool  initialized;  // 是否已初始化
} BandPassFilter_t;

/**
 * @brief High pass filter structure
 */
typedef struct
{
    float alpha;        // Filter coefficient
    float prev_input;   // Previous input value
    float prev_output;  // Previous output value
    bool  initialized;  // Initialization flag
} HighPassFilter_t;

/**
 * @brief Band stop (notch) filter structure
 */
typedef struct
{
    // 滤波器系数
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;

    // 延迟线（存储历史输入输出）
    float x1;  // 前一个输入
    float x2;  // 前两个输入
    float y1;  // 前一个输出
    float y2;  // 前两个输出
} BandStopFilter_t;

typedef struct
{
    // 滤波器系数
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    // 历史数据
    float x1;  // 输入历史值1
    float x2;  // 输入历史值2
    float y1;  // 输出历史值1
    float y2;  // 输出历史值2

    bool initialized;  // 是否已初始化
} IIR2ndFilter_t;

// Low pass filter functions
void IIR1stFilter_Init(IIR1stFilter_t* filter,
                       float           cutoff_freq,
                       float           sample_freq);
/**
 * @brief Update low pass filter with improved
 * initialization
 * @param filter: pointer to IIR1stFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
static inline float IIR1stFilter_Update(IIR1stFilter_t* filter, float x)
{
    float y = filter->alpha * filter->prev_output
              + (1.0F - filter->alpha) * x;
    filter->prev_output = y;
    return y;
}

void IIR1stFilter_Reset(IIR1stFilter_t* filter);

// High pass filter functions
void HighPassFilter_Init(HighPassFilter_t* filter,
                         float             cutoff_freq,
                         float             sample_freq);
/**
 * @brief Update high pass filter
 * @param filter: pointer to HighPassFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
static inline float HighPassFilter_Update(HighPassFilter_t* filter,
                                          float             input)
{
    if (filter == NULL)
    {
        return input;
    }

    // Initialize with first input value
    if (!filter->initialized)
    {
        filter->prev_input  = input;
        filter->prev_output = 0.0f;
        filter->initialized = true;
        return 0.0f;
    }

    // High pass filter equation: y[n] = α * (y[n-1] + x[n] - x[n-1])
    filter->prev_output
        = filter->alpha
          * (filter->prev_output + input - filter->prev_input);
    filter->prev_input = input;

    return filter->prev_output;
}
void HighPassFilter_Reset(HighPassFilter_t* filter);

// Band pass filter functions
void BandPassFilter_Init(BandPassFilter_t* filter,
                         float             sampleRate,
                         float             centerFreq,
                         float             bandwidth);

/**
 * @brief Update band pass filter
 * @param filter: pointer to BandPassFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
static inline float BandPassFilter_Update(BandPassFilter_t* filter,
                                          float             input)
{
    if (filter == NULL)
    {
        return input;  // 如果滤波器指针无效，返回原始输入
    }

    // 计算当前输出（二阶IIR滤波器公式）
    float output = filter->b0 * input + filter->b1 * filter->x1
                   + filter->b2 * filter->x2 - filter->a1 * filter->y1
                   - filter->a2 * filter->y2;

    // 更新延迟线（移位操作）
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}
void BandPassFilter_Reset(BandPassFilter_t* filter);

// Band stop (notch) filter functions
void BandStopFilter_Init(BandStopFilter_t* filter,
                         float             sampleRate,
                         float             centerFreq,
                         float             bandwidth);

/**
 * @brief Update band stop filter (parallel low-pass and
 * high-pass combination)
 * @param filter: pointer to BandStopFilter_t structure
 * @param input: input value
 * @return filtered output value
 */
static inline float BandStopFilter_Update(BandStopFilter_t* filter,
                                          float             input)
{
    if (filter == NULL)
    {
        return input;  // 如果滤波器指针无效，返回原始输入
    }

    // 计算当前输出（二阶IIR滤波器公式）
    float output = filter->b0 * input + filter->b1 * filter->x1
                   + filter->b2 * filter->x2 - filter->a1 * filter->y1
                   - filter->a2 * filter->y2;

    // 更新延迟线（移位操作）
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

void BandStopFilter_Reset(BandStopFilter_t* filter);

void IIR2ndFilter_Init(IIR2ndFilter_t* filter,
                       float           cutoff_freq,
                       float           sample_freq);

static inline float IIR2ndFilter_Update(IIR2ndFilter_t* filter,
                                        float           input)
{
    if (filter == NULL)
    {
        return input;  // 无效指针处理
    }

    // 计算当前输出
    float output = filter->b0 * input + filter->b1 * filter->x1
                   + filter->b2 * filter->x2 - filter->a1 * filter->y1
                   - filter->a2 * filter->y2;

    // 更新历史数据（移位操作）
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}

void IIR2ndFilter_Reset(IIR2ndFilter_t* filter);

#endif /* __FILTER_H__ */
