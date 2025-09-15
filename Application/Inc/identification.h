#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H
#include "stdbool.h"
#include "stdint.h"

#define CURRENT_FILTER_ALPHA 0.1F
#define CURRENT_LIMIT 10.0F  // 最大允许电流（A）
#define CURRENT_RATIO 0.8F   // 估 Rs 时允许的最大电流
#define RS_THRESHOLD 0.01F   // Rs 收敛阈值（Ohm）
#define VOLTAGE_STEP 1.0F    // 每步电压增量（V）
#define MAX_STEPS 20         // 最大步数
#define HOLD_CYCLES 15000    // 每步保持周期数

/* --------- 参数区（按目标 MCU 调整这些数值以节省 RAM） --------- */
#define SAMPLE_CAPACITY 512  // 缓冲区大小（samples），可改为 1024 / 4096
#define MAX_STEPS 20         // 最多 20 个 Imax 步
#define REPEAT_TIMES 3       // 每个 Imax 重复 3 次

/* ----------公式系数------------------ */
#define S 5
#define T 1
#define U 1
#define V 0

typedef struct
{
  float Imax_value;
  float avg_max_psi;  // 选取的 N=SELECT_CYCLES 周期中每周期最大 psi 的平均值
  int cycles_used;    // SELECT_CYCLES (通常)
} ImaxResult_t;

typedef struct
{
  float J;
  float R2;
} Residual_t;

typedef struct
{
  float J[2];
  float R2[2];
} ResidualDQ_t;

typedef struct
{
  float ad0;
  float add;
  float aq0;
  float aqq;
  float adq;
  Residual_t D;
  Residual_t Q;
  ResidualDQ_t DQ;
} LLS_Result_t;

/* 状态机状态 */
typedef enum
{
  WAIT = 0,
  EST_RS,          // 估定 Rs
  PROCESS,         // 处理本步数据
  INJECT_COLLECT,  // 注入并收集样本
  NEXT_I,          // 切换到下一个 Imax
  LLS,
  PENDING,
  DONE
} ExpState_e;

/* 你已有的结构（若有差别请据实修改） */
typedef enum
{
  INJECT_D = 0,
  INJECT_Q,
  INJECT_DQ
} Inj_Mode_e;

typedef struct
{
  bool State;
  float Vd, Vq;  // 当前输出电压（SquareWaveGenerater 更新）
  float Ud_amp;  // 方波幅值（注入幅值）
  float Uq_amp;
  float Imax;          // 切换判定阈值
  Inj_Mode_e mode;     // 注入模式（D 轴 / Q 轴 / DQ 轴）
  int8_t inj_state_d;  // D 轴注入状态（+1 或 -1）
  int8_t inj_state_q;  // Q 轴注入状态（+1 或 -1）
} VoltageInjector_t;

/* 实验主体结构（静态分配） */
typedef struct
{
  // config
  float Ts;             // 采样周期 (s)
  float Rs_est;         // Rs 的估计值（由 Estimate_Rs 得到）
  int sample_capacity;  // <= SAMPLE_CAPACITY
  int max_steps;        // <= MAX_STEPS

  // 重复控制
  int repeat_times;   // 每个 Imax 要重复多少次
  int repeat_count;   // 当前已经完成的次数
  float sum_max_psi;  // 累积的 max_psi
  float sum_max_I;    // 累积的 max_I

  int wait_edges;  // 开始采集前需要等待的边沿数

  // sweeping
  int start_I;   // 起始 Imax (int A)
  int final_I;   // 终止 Imax
  int step_dir;  // -1 表示从 start 向 final 递减，+1 表示递增

  // buffers
  float Ud_buf[SAMPLE_CAPACITY];
  float Id_buf[SAMPLE_CAPACITY];
  float Uq_buf[SAMPLE_CAPACITY];
  float Iq_buf[SAMPLE_CAPACITY];
  float psi_d_buf[SAMPLE_CAPACITY];
  float psi_q_buf[SAMPLE_CAPACITY];

  // 交叉耦合累加器（用于最终求解 adq）
  float cq_Sxx;  // sum of x^2 (x1^2 + x2^2 over all used samples)
  float cq_Sxy;  // sum of x*y (x1*id_res + x2*iq_res)
  uint8_t cq_N;  // count of used sample-rows (2 per k if both rows used)

  // 用于计算 J / R^2
  float sum_id;   // sum of id (for mean)
  float sum_id2;  // sum of id^2
  float sum_iq;   // sum of iq
  float sum_iq2;  // sum of iq^2

  float sum_eps_id2;  // sum of squared residuals id (Jd)
  float sum_eps_iq2;  // sum of squared residuals iq (Jq)
  uint16_t count_id;  // number of id samples included
  uint16_t count_iq;  // number of iq samples included

  // fill pointers
  int pos;          // 当前写入位置
  int edge_idx[2];  // 切换点索引, 0: 起点, 1: 终点
  int edge_count;

  float last_Vd;
  float last_Vq;

  // results
  ImaxResult_t results[MAX_STEPS];
  LLS_Result_t LLS;
  int step_index;  // 已完成的 step 数

  // runtime
  ExpState_e state;
  VoltageInjector_t inj;  // 指向外部注入器
  float inject_amp;       // 注入幅值（外部传入，Ud_amp / Uq_amp）
  bool Running;           // 是否正在运行（非 DONE）
} FluxExperiment_t;

void Experiment_Step(FluxExperiment_t* exp, float Id, float Iq, float* Ud, float* Uq);
void Experiment_Init(FluxExperiment_t* exp, float Ts, int sample_capacity, int repeat_times,
                     int max_steps, int start_I, int final_I, int step_dir, float inject_amp);
// void MTPA_Init(void);
// void MTPA_SetCurrent(float current);
// float MTPA_GetCurrent(void);

#endif /* MTPA_H */
