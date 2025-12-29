/**
 * @file sensorless_interface.h
 * @brief 无传感器控制接口层
 * @author FRECON
 * @date 2025年7月28日
 * @version 2.0
 *
 * 该文件提供无传感器控制的全局变量接口，类似FOC算法的设计模式
 */

#ifndef __SENSORLESS_INTERFACE_H__
#define __SENSORLESS_INTERFACE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "motor.h"
#include "pid.h"
#include "reciprocal.h"
#include "theta_calc.h"
#include "transformation.h"

/**
 * @brief 无传感器控制方法枚举
 */
typedef enum {
    FLYING       = 0, /*!< 无传感器控制初始辨识 */
    LES_OBSERVER = 1, /*!< 线性扩张状态观测器 */
    HF_INJECTION = 2, /*!< 高频注入 */
} sensorless_method_t;

/**
 * @brief 无传感器控制状态枚举
 */
typedef enum {
    SENSORLESS_STATE_STOPPED = 0, /*!< 停止状态 */
    SENSORLESS_STATE_STARTING,    /*!< 启动状态 */
    SENSORLESS_STATE_RUNNING,     /*!< 运行状态 */
    SENSORLESS_STATE_ERROR        /*!< 错误状态 */
} sensorless_state_t;

typedef struct {
    float hysteresis;    // 滞环宽度
    float switch_speed;  // 切换速度
} Sensorless_Param_t;

extern float Sensorless_PLL_Wc;

bool Sensorless_Set_SampleTime(const SystemTimeConfig_t* config);

bool Sensorless_Initialization(const Sensorless_Param_t* param);

bool Sensorless_Set_SpeedFilter(float cutoff_freq, float sample_freq);

bool Sensorless_Set_PidParams(const PID_Handler_t* pid_handler);

bool Sensorless_Set_MotorParams(const MotorParam_t* motor_param);

bool Sensorless_Set_Voltage(Clark_t voltage);

bool Sensorless_Set_Current(Clark_t current);

void Sensorless_Set_SpeedFdbk(float fdbk);

void Sensorless_Set_SpeedRef(float ref);

void Sensorless_Set_SpeedTarget(float ref);

//void Sensorless_Set_Angle(float angle);

bool Sensorless_Set_ResetFlag(bool enabled);

bool Sensorless_Get_Reset(void);

bool Sensorless_Set_Method(sensorless_method_t method, bool enable);

sensorless_method_t Sensorless_Get_Method(void);

Clark_t Sensorless_Get_SmoEmf(void);

bool Sensorless_Calculate_Err(AngleResult_t result);

AngleResult_t Sensorless_Get_Error(void);

AngleResult_t Sensorless_Update_Position(void);

bool Sensorless_Calculate(void);

Park_t Sensorless_Inject_Voltage(Park_t voltage);

Clark_t Sensorless_FilterCurrent(Clark_t current);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORLESS_INTERFACE_H__ */
