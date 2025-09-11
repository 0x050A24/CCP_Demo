#ifndef MTPA_H
#define MTPA_H
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
#define CAPTURE_CYCLES 20    // 每步采集的最大周期数（尽量 >= 10）,选取1以节省内存。一个周期约400点
#define SELECT_CYCLES 10     // 从 CAPTURE_CYCLES 中选取的中心周期数量（取中间10）

/* ------------------------------------------------------------------ */


typedef struct
{
  float Imax_value;
  float avg_max_psi;  // 选取的 N=SELECT_CYCLES 周期中每周期最大 psi 的平均值
  int cycles_used;    // SELECT_CYCLES (通常)
} ImaxResult_t;

typedef struct {
    float ad0;
    float add;
    float aq0;
    float aqq;
    float adq;
} LLS_Result_t;

/* 状态机状态 */
typedef enum
{
  WAIT = 0,
  EST_RS,          // 估定 Rs
  INJECT_COLLECT,  // 注入并收集样本
  PROCESS,         // 处理本步数据
  NEXT_I,          // 切换到下一个 Imax
  LLS,
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
  float Imax;           // 切换判定阈值
  Inj_Mode_e mode;      // 注入模式（D 轴 / Q 轴 / DQ 轴）
  int8_t inj_state_d;  // D 轴注入状态（+1 或 -1）
  int8_t inj_state_q;  // Q 轴注入状态（+1 或 -1）
  uint32_t Count;       // 采样计数（外部自增）
} VoltageInjector_t;

/* 实验主体结构（静态分配） */
typedef struct
{
  // config
  float Ts;             // 采样周期 (s)
  float Rs_est;         // Rs 的估计值（由 Estimate_Rs 得到）
  int sample_capacity;  // <= SAMPLE_CAPACITY
  int capture_cycles;   // CAPTURE_CYCLES（<= CAPTURE_CYCLES 宏）
  int select_cycles;    // SELECT_CYCLES
  int max_steps;        // <= MAX_STEPS

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

  // fill pointers
  int pos;                           // 当前写入位置
  int edge_idx[CAPTURE_CYCLES + 4];  // 切换点索引（需要 cycles+1）
  int edge_count;

  float last_Vd;
  float last_Vq;

  // results
  ImaxResult_t results[MAX_STEPS];
  int step_index;  // 已完成的 step 数

  // runtime
  ExpState_e state;
  VoltageInjector_t* inj;  // 指向外部注入器
  float inject_amp;        // 注入幅值（外部传入，Ud_amp / Uq_amp）
  bool Running;            // 是否正在运行（非 DONE）
} FluxExperiment_t;

void Experiment_Step(FluxExperiment_t* exp, float Id, float Iq, float* Ud, float* Uq);
void Experiment_Init(FluxExperiment_t* exp, VoltageInjector_t* inj, float Ts, int sample_capacity,
                     int capture_cycles, int select_cycles, int max_steps, int start_I, int final_I,
                     int step_dir, float inject_amp);
// void MTPA_Init(void);
// void MTPA_SetCurrent(float current);
// float MTPA_GetCurrent(void);

#endif /* MTPA_H */
