/*********************************************************************/
/*                        初始化参数头文件                              */
/*                                                                   */
/* 文件名: parameters.h                                               */
/* 描述: 包含电机控制所需的所有初始化参数和常量定义                          */
/*********************************************************************/

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

/*********************************************************************/
/*                        门极驱动极性定义                              */
/*********************************************************************/
#ifndef GATE_POLARITY_HIGH_ACTIVE
#    ifndef GATE_POLARITY_LOW_ACTIVE
#        define GATE_POLARITY_LOW_ACTIVE
#    endif
#endif

#if !defined(GATE_POLARITY_HIGH_ACTIVE) \
    && !defined(GATE_POLARITY_LOW_ACTIVE)
#    error \
        "请在foc.h中定义GATE_POLARITY_HIGH_ACTIVE或GATE_POLARITY_LOW_ACTIVE"
#endif

/*********************************************************************/
/*                        位置传感器类型定义                            */
/*********************************************************************/
#ifndef RESOLVER_POSITION
#    ifndef ENCODER_POSITION
#        define ENCODER_POSITION
#    endif
#endif

#if !defined(RESOLVER_POSITION) && !defined(ENCODER_POSITION)
#    error "请在foc.h中定义RESOLVER_POSITION或ENCODER_POSITION"
#endif

/*********************************************************************/
/*                        时钟参数配置                                 */
/*********************************************************************/
/* 主频配置 */
#define MCU_MAIN_FREQ 120000000U /* MCU主频：120MHz */

/* 主定时器配置 */
#define MAIN_INT_TIMER_PRESCALER 0 /* 预分频器：不分频 */
#define MAIN_INT_TIMER_PERIOD \
    12000 /* 周期值：12000 (10kHz = 120MHz/12000/2) */
#define MAIN_INT_TIMER_DEADTIME_PERIOD \
    2000 /* 死区时间：2us (2us = 120MHz/12000/2*2000) */

/* 主循环(电流环)频率配置 */
#define MAIN_LOOP_FREQ                                 \
    (MCU_MAIN_FREQ / (MAIN_INT_TIMER_PRESCALER + 1.0F) \
     / MAIN_INT_TIMER_PERIOD / 2)              /* 10kHz */
#define MAIN_LOOP_TIME (1.0F / MAIN_LOOP_FREQ) /* 100us */

/* 转速环频率配置 */
#define SPEED_LOOP_PRESCALER 10.0F /* 转速环分频系数 */
#define SPEED_LOOP_FREQ \
    (MAIN_LOOP_FREQ / SPEED_LOOP_PRESCALER)      /* 1kHz */
#define SPEED_LOOP_TIME (1.0F / SPEED_LOOP_FREQ) /* 1ms */

/*********************************************************************/
/*                        电机物理参数                                 */
/*********************************************************************/
/* 电机电气参数 */
#define MOTOR_RS   0.65F   /* 定子电阻：0.65 Ω */
#define MOTOR_LD   100E-3F /* d轴电感：100 mH */
#define MOTOR_LQ   16E-3F  /* q轴电感：16 mH */
#define MOTOR_FLUX 0.0F    /* 永磁体磁链：0.1 Wb */
#define MOTOR_PN   2.0F    /* 电机极对数：2 */

/* 位置传感器配置 */
#ifdef RESOLVER_POSITION
#    define MOTOR_POSITION_SCALE (65536U - 1U) /* 旋变分辨率：16位 */
#endif
#ifdef ENCODER_POSITION
#    define MOTOR_POSITION_SCALE \
        (10000U - 1U) /* 编码器分辨率：10000线 */
#endif

/* 角度计算因子 */
#ifdef RESOLVER_POSITION
#    define MOTOR_THETA_FACTOR \
        (M_2PI / ((MOTOR_POSITION_SCALE + 1) * MOTOR_RESOLVER_PN))
#    define MOTOR_POSITION_OFFSET 39293.0F /* 位置传感器零点偏置 */
#endif
#ifdef ENCODER_POSITION
#    define MOTOR_THETA_FACTOR \
        (M_2PI / (float)(MOTOR_POSITION_SCALE + 1))
#    define MOTOR_POSITION_OFFSET 4260.0F /* 位置传感器零点偏置 */
#endif

#define MOTOR_RESOLVER_PN 1.0F /* 旋变极对数 */

/*********************************************************************/
/*                        保护参数配置                                 */
/*********************************************************************/
/* 电压保护参数 */
#define PROTECT_VOLTAGE_RATE        560.0F /* 额定电压：560V */
#define PROTECT_VOLTAGE_FLUCTUATION 160.0F /* 允许电压波动：±60V */

/* 电流和温度保护参数 */
#define PROTECT_CURRENT_MAX 30.0F /* 最大电流限制：30A */
#define PROTECT_TEMPERATURE 80.0F /* 最高温度限制：80℃ */

/*********************************************************************/
/*                        FOC控制参数配置                              */
/*********************************************************************/
/* 转速斜坡控制参数 */
#define RAMP_SPEED_SLOPE     200.0F   /* 速度变化率限制：100 rpm/s */
#define RAMP_SPEED_LIMIT_MAX 1800.0F  /* 最大转速限制：1800 rpm */
#define RAMP_SPEED_LIMIT_MIN -1800.0F /* 最小转速限制：-1800 rpm */
#define RAMP_SPEED_TIME      (SPEED_LOOP_TIME) /* 转速环采样周期 */

/* 转速环PID参数配置 */
#define PID_SPEED_LOOP_KP 0.016F /* 转速环比例系数 */
#define PID_SPEED_LOOP_KI 0.060F /* 转速环积分系数 */
#define PID_SPEED_LOOP_KD 0.00F  /* 转速环微分系数 */

/* 转速环输出限制 */
#define PID_SPEED_LOOP_MAX_OUTPUT \
    (0.8F * PROTECT_CURRENT_MAX) /* q轴最大电流限制 */
#define PID_SPEED_LOOP_MIN_OUTPUT (-1.0F * PID_SPEED_LOOP_MAX_OUTPUT)
#define PID_SPEED_LOOP_INTEGRAL_LIMIT \
    PID_SPEED_LOOP_MAX_OUTPUT                 /* 积分限幅值 */
#define PID_SPEED_LOOP_TIME (SPEED_LOOP_TIME) /* 转速环采样周期 */

/* 电流环d轴PID参数配置 */
#define PID_CURRENT_D_LOOP_KP 73.8274273F   /* d轴比例系数 */
#define PID_CURRENT_D_LOOP_KI 408.40704496F /* d轴积分系数 */
#define PID_CURRENT_D_LOOP_KD 0.00F         /* d轴微分系数 */

/* d轴输出限制 */
#define PID_CURRENT_D_LOOP_MAX_OUTPUT 300.0F /* 最大输出电压：Udc/√3 */
#define PID_CURRENT_D_LOOP_MIN_OUTPUT \
    (-1.0F * PID_CURRENT_D_LOOP_MAX_OUTPUT)
#define PID_CURRENT_D_LOOP_INTEGRAL_LIMIT \
    PID_CURRENT_D_LOOP_MAX_OUTPUT                /* 积分限幅值 */
#define PID_CURRENT_D_LOOP_TIME (MAIN_LOOP_TIME) /* d轴采样周期 */

/* 电流环q轴PID参数配置 */
#define PID_CURRENT_Q_LOOP_KP 27.646015F    /* q轴比例系数 */
#define PID_CURRENT_Q_LOOP_KI 408.40704496F /* q轴积分系数 */
#define PID_CURRENT_Q_LOOP_KD 0.00F         /* q轴微分系数 */

/* q轴输出限制 */
#define PID_CURRENT_Q_LOOP_MAX_OUTPUT 400.0F /* 最大输出电压：Udc/√3 */
#define PID_CURRENT_Q_LOOP_MIN_OUTPUT \
    (-1.0F * PID_CURRENT_Q_LOOP_MAX_OUTPUT)
#define PID_CURRENT_Q_LOOP_INTEGRAL_LIMIT \
    PID_CURRENT_Q_LOOP_MAX_OUTPUT                /* 积分限幅值 */
#define PID_CURRENT_Q_LOOP_TIME (MAIN_LOOP_TIME) /* q轴采样周期 */

/*********************************************************************/
/*                        无位置运行参数配置                            */
/*********************************************************************/
/* 无位置传感器策略控制 */
#define SENSORLESS_HYSTERESIS   60.0F  /* 无传感器滞环宽度：50rpm */
#define SENSORLESS_SWITCH_SPEED 400.0F /* 无传感器切换速度：460rpm */

/* 无位置PLL跟踪器参数 */
#define SENSORLESS_PLL_WC         25.0F  /* PLL带宽 */
#define SENSORLESS_PLL_KP         50.0F  /* PLL比例系数 */
#define SENSORLESS_PLL_KI         625.0F /* PLL积分系数 */
#define SENSORLESS_PLL_KD         0.0F   /* PLL微分系数 */
#define SENSORLESS_PLL_MAX_OUTPUT 700.0F /* PLL最大输出 */
#define SENSORLESS_PLL_MIN_OUTPUT \
    (-1 * SENSORLESS_PLL_MAX_OUTPUT) /* PLL最小输出 */
#define SENSORLESS_PLL_INTEGRAL_LIMIT \
    SENSORLESS_PLL_MAX_OUTPUT /* PLL积分限幅值 */

/* 高频信号注入参数 */
#define HF_INJECTION_FREQ 500.0F /* 注入信号频率：500Hz */
#define HF_INJECTION_AMP  40.0F  /* 注入信号幅值：40V */

/* 高频信号注入带通滤波器参数 */
#define HFI_RESPONSE_FREQ      HF_INJECTION_FREQ /* 中心频率 */
#define HFI_RESPONSE_BANDWIDTH 40.0F          /* 滤波器频带宽度：40Hz */
#define HFI_SAMPLE_TIME        MAIN_LOOP_TIME /* 采样周期：与主循环相同 */
#define HFI_SAMPLE_FREQ        MAIN_LOOP_FREQ /* 采样周期：与主循环相同 */

/* 高频信号注入法低通滤波器参数 */
#define HFI_LOW_PASS_CUTOFF_FREQ 100.0F /* 误差信号截止频率：100Hz */

/* 高频注入PLL跟踪器参数 */
#define HFI_PLL_KP             12E2F  /* PLL比例系数 */
#define HFI_PLL_KI             40E3F  /* PLL积分系数 */
#define HFI_PLL_KD             0.0F   /* PLL微分系数 */
#define HFI_PLL_MAX_OUTPUT     500.0F /* PLL最大输出 */
#define HFI_PLL_MIN_OUTPUT     (-1 * HFI_PLL_MAX_OUTPUT) /* PLL最小输出 */
#define HFI_PLL_INTEGRAL_LIMIT HFI_PLL_MAX_OUTPUT /* PLL积分限幅值 */

/* LESO参数 */
#define LESO_WC_GAIN 8.0F    /* 观测器带宽系数 */
#define LESO_WC_MAX  4000.0F /* 观测器带宽最大值 */
#define LESO_WC_MIN  600.0F  /* 观测器带宽最小值 */

/* 滑模观测器PLL跟踪器参数 */
#define SMO_PLL_KP             600.0F  /* PLL比例系数 */
#define SMO_PLL_KI             90E3F   /* PLL积分系数 */
#define SMO_PLL_KD             0.0F    /* PLL微分系数 */
#define SMO_PLL_MAX_OUTPUT     4000.0F /* PLL最大输出 */
#define SMO_PLL_MIN_OUTPUT     (-1 * SMO_PLL_MAX_OUTPUT) /* PLL最小输出 */
#define SMO_PLL_INTEGRAL_LIMIT SMO_PLL_MAX_OUTPUT /* PLL积分限幅值 */

/* 滑模观测器低通滤波器参数 */
#define SMO_LPF_CUTOFF_FREQ 500.0F /* 低通滤波器截止频率：500Hz */
#define SMO_LPF_ORDER       2      /* 低通滤波器阶数：2 */
#define SMO_SAMPLING_FREQ \
    MAIN_LOOP_FREQ /* 采样频率：与主循环频率相同 */

/*********************************************************************/
/*                        Buffer参数配置                            */
/*********************************************************************/
#define BUFFER_CAPACITY  10U /* 默认缓冲区容量 */
#define BUFFER_PRESCALER 1U  /* 默认缓冲区预分频器 */

#endif
