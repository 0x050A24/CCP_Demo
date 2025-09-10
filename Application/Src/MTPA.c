#include "MTPA.h"
#include "math.h"
#include "stdint.h"

// flux_injector.c
// 依赖: stdlib.h, stdint.h, stdbool.h, string.h, math.h

// flux_rs_inject.c
// 状态机：先估 Rs，然后在 Imax 从 start_I 以 1A 步进到 min_I（递减）范围内
// 每个 Imax 采集 CAPTURE_CYCLES 周期（默认 20），从中取中间 10 周期计算平均最大磁链（去均值后的
// psi） 无 malloc，所有 buffer 静态分配

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Estimate_Rs 与 SquareWaveGenerater 原型（你已有的实现） */
static inline bool Estimate_Rs(float Current, float* Voltage_out, float* Rs);

/* 结果结构：每个 Imax 只保存最终的 avg_max_psi（和 Imax 值） */

// 用 volatile 保证编译器不会优化掉，A2L 能映射到这些数组
volatile float g_results_Imax[MAX_STEPS];
volatile float g_results_avgmax[MAX_STEPS];
volatile uint32_t g_results_cycles[MAX_STEPS];

// 当前步索引
volatile uint32_t g_result_index = 0;

// 保存一次结果
void save_result(float Imax, float avgmax, uint32_t cycles)
{
  if (g_result_index < MAX_STEPS)
  {
    g_result_index++;
    g_results_Imax[g_result_index] = Imax;
    g_results_avgmax[g_result_index] = avgmax;
    g_results_cycles[g_result_index] = cycles;
  }
}

/* ---------- 初始化 ---------- */
void Experiment_Init(FluxExperiment_t* exp, VoltageInjector_t* inj, float Ts, int sample_capacity,
                     int capture_cycles, int select_cycles, int max_steps, int start_I, int final_I,
                     int step_dir, float inject_amp)
{
  // clip params
  if (sample_capacity > SAMPLE_CAPACITY) sample_capacity = SAMPLE_CAPACITY;
  if (capture_cycles > CAPTURE_CYCLES) capture_cycles = CAPTURE_CYCLES;
  if (select_cycles > SELECT_CYCLES) select_cycles = SELECT_CYCLES;
  if (max_steps > MAX_STEPS) max_steps = MAX_STEPS;

  memset(exp, 0, sizeof(FluxExperiment_t));
  exp->Ts = Ts;
  exp->sample_capacity = sample_capacity;
  exp->capture_cycles = capture_cycles;
  exp->select_cycles = select_cycles;
  exp->max_steps = max_steps;
  exp->start_I = start_I;
  exp->final_I = final_I;
  exp->step_dir = (step_dir >= 0) ? 1 : -1;
  exp->inject_amp = inject_amp;
  exp->Running = false;
  exp->inj = inj;
  exp->pos = 0;
  exp->edge_count = 0;
  exp->step_index = 0;
  exp->state = WAIT;
  exp->last_Vd = inj ? inj->Vd : 0.0f;
  exp->last_Vq = inj ? inj->Vq : 0.0f;
}

void Experiment_Step(FluxExperiment_t* exp, float Id, float Iq, float* Ud, float* Uq)
{
  if (!exp) return;

  switch (exp->state)
  {
    case WAIT:
      *Ud = 0.0f;
      *Uq = 0.0f;
      //   if (exp->Running && exp->Rs_est == 0.0F)
      //   {
      //     exp->state = EST_RS;
      //   }
      //   if (exp->Running && exp->Rs_est != 0.0F)
      //   {
      //     exp->state = INJECT_COLLECT;
      //   }

      return;

    case EST_RS:
    {
      float Voltage_out = 0.0f;
      float Rs_tmp = 0.0f;

      bool done = Estimate_Rs(Id, &Voltage_out, &Rs_tmp);

      // 直接把 Rs 估计电压输出到电机
      *Ud = Voltage_out;
      *Uq = 0.0f;

      if (done)
      {
        // 保存 Rs
        exp->Rs_est = Rs_tmp;

        // 关闭电压输出
        *Ud = 0.0f;
        *Uq = 0.0f;

        // 初始化方波注入器
        if (exp->inj)
        {
          exp->inj->Ud_amp = 0.0F;
          exp->inj->Uq_amp = 0.0F;
          exp->inj->Imax = 0.0F;
          exp->inj->Count = 0;
          exp->inj->State = false;
        }

        exp->pos = 0;
        exp->edge_count = 0;
        exp->last_Vd = exp->inj ? exp->inj->Vd : 0.0f;
        exp->last_Vq = exp->inj ? exp->inj->Vq : 0.0f;
        // exp->state = INJECT_COLLECT;
        exp->state = WAIT;
      }
      break;
    }

    case INJECT_COLLECT:
    {
      float Ud_out = 0.0f, Uq_out = 0.0f;

      if (exp->inj->State == false)
      {
        Ud_out = 0.0F;
        Uq_out = 0.0F;
        *Ud = 0.0F;
        *Uq = 0.0F;
        break;
      }

      if (exp->inj->State)
      {
        // --- 方波注入 D 轴 ---
        if (exp->inj->mode == INJECT_D || exp->inj->mode == INJECT_DQ)
        {
          if (Id >= exp->inj->Imax)
          {
            exp->inj->inj_state_d = -1;
          }
          else if (Id <= -exp->inj->Imax)
          {
            exp->inj->inj_state_d = +1;
          }
          Ud_out = (exp->inj->inj_state_d >= 0) ? exp->inj->Ud_amp : -exp->inj->Ud_amp;
        }

        // --- 方波注入 Q 轴 ---
        if (exp->inj->mode == INJECT_Q || exp->inj->mode == INJECT_DQ)
        {
          if (Iq >= exp->inj->Imax)
          {
            exp->inj->inj_state_q = -1;
          }
          else if (Iq <= -exp->inj->Imax)
          {
            exp->inj->inj_state_q = +1;
          }
          Uq_out = (exp->inj->inj_state_q >= 0) ? exp->inj->Uq_amp : -exp->inj->Uq_amp;
        }

        // 本次输出电压
        *Ud = Ud_out;
        *Uq = Uq_out;
      }

      // --- 边沿检测 ---
      if (exp->inj->mode == INJECT_D || exp->inj->mode == INJECT_DQ)
      {
        if (Ud_out == -exp->last_Vd)
        {
          if (exp->edge_count < (exp->capture_cycles + 2))
          {
            exp->edge_idx[exp->edge_count++] = exp->pos;
          }
        }
      }
      if (exp->inj->mode == INJECT_Q)
      {
        if (Uq_out == -exp->last_Vq)
        {
          if (exp->edge_count < (exp->capture_cycles + 2))
          {
            exp->edge_idx[exp->edge_count++] = exp->pos;
          }
        }
      }

      // --- 存 buffer ---
      if (exp->edge_count != 0)
      {
        if (exp->pos < exp->sample_capacity)
        {
          int idx = exp->pos;
          exp->Ud_buf[idx] = exp->last_Vd;
          exp->Id_buf[idx] = Id;
          exp->Uq_buf[idx] = exp->last_Vq;
          exp->Iq_buf[idx] = Iq;

          // 积分（前向 Euler）
          if (idx == 0)
          {
            exp->psi_d_buf[idx] = 0.0f;
            exp->psi_q_buf[idx] = 0.0f;
          }
          else
          {
            int prev = idx - 1;
            float integrand_d = (exp->last_Vd - exp->Rs_est * Id);
            float integrand_q = (exp->last_Vq - exp->Rs_est * Iq);
            exp->psi_d_buf[idx] = exp->psi_d_buf[prev] + exp->Ts * integrand_d;
            exp->psi_q_buf[idx] = exp->psi_q_buf[prev] + exp->Ts * integrand_q;
          }

          exp->pos++;
        }
        else
        {
          // buffer 满，进入处理
          exp->state = PROCESS;
          exp->inj->State = false;
          Ud_out = 0.0F;
          Uq_out = 0.0F;
          *Ud = 0.0F;
          *Uq = 0.0F;
        }
      }

      // --- 检查是否捕获到足够的周期 ---
      if (exp->edge_count >= (exp->capture_cycles + 1))
      {
        exp->state = PROCESS;
        exp->inj->State = false;
        *Ud = 0.0F;
        *Uq = 0.0F;
      }
      exp->last_Vd = Ud_out;  // 最后更新
      exp->last_Vq = Uq_out;
      break;
    }

    case PROCESS:
    {
      // 处理当前 step 的数据：要求至少 capture_cycles 个周期（否则视为失败）
      int pairs = exp->edge_count - 1;
      if (pairs < exp->select_cycles)
      {
        // 数据不足：直接结束当前 step（可选择重做或标记失败）
        // 这里我们把该步标为使用尽可能多的周期（若 >=10），否则跳过保存
        if (pairs < 1)
        {
          // 没有周期，直接结束
          exp->state = NEXT_I;
          break;
        }
      }

      // 容许的可用周期数 = min(pairs, capture_cycles)
      int cycles_avail = pairs;
      if (cycles_avail > exp->capture_cycles) cycles_avail = exp->capture_cycles;

      // 选择中间 select_cycles 周期（如果 cycles_avail < select_cycles，则使用全部）
      int use_cycles = exp->select_cycles;
      if (cycles_avail < use_cycles) use_cycles = cycles_avail;

      int start_cycle = 0;
      if (cycles_avail > use_cycles)
      {
        // center the selection
        start_cycle = (cycles_avail - use_cycles) / 2;
      }
      else
      {
        start_cycle = 0;
      }

      // 计算每个所选周期去均值后的 psi 最大值（以 d 轴为例）
      float sum_max_psi = 0.0f;
      int counted = 0;
      for (int c = 0; c < use_cycles; ++c)
      {
        int cyc_idx = start_cycle + c;
        int s_idx = exp->edge_idx[cyc_idx];
        int e_idx = exp->edge_idx[cyc_idx + 1];
        if (e_idx <= s_idx + 1) continue;
        // 计算均值
        float s = 0.0f;
        int cnt = 0;
        for (int i = s_idx; i < e_idx; ++i)
        {
          s += exp->psi_d_buf[i];
          cnt++;
        }
        if (cnt == 0) continue;
        float mean = s / (float)cnt;
        // 找到该周期内去均值后的最大 psi
        float max_psi = -1e30f;
        for (int i = s_idx; i < e_idx; ++i)
        {
          float psi_c = exp->psi_d_buf[i] - mean;
          if (psi_c > max_psi) max_psi = psi_c;
        }
        if (max_psi > -1e29f)
        {
          sum_max_psi += max_psi;
          counted++;
        }
      }

      if (counted > 0)
      {
        exp->results[exp->step_index].Imax_value = exp->inj ? exp->inj->Imax : 0.0f;
        exp->results[exp->step_index].avg_max_psi = sum_max_psi / (float)counted;
        exp->results[exp->step_index].cycles_used = counted;
        exp->step_index++;
        save_result(exp->inj ? exp->inj->Imax : 0.0f, sum_max_psi / (float)counted, counted);
      }
      else
      {
        // 本步没计算出有效最大值（可记录失败），这里忽略保存
      }

      exp->state = NEXT_I;
      break;
    }

    case NEXT_I:
    {
      if (exp->inj)
      {
        int curI = (int)exp->inj->Imax;
        int newI = 0;

        if (exp->step_index == 0)
        {
          // 第一次，直接跳到 start_I
          newI = exp->start_I;
        }
        else
        {
          // 后续按 step_dir 增减
          newI = curI + exp->step_dir;
        }

        // 检查是否超出范围
        bool finished = false;
        if (exp->step_dir < 0)
        {
          if (newI < exp->final_I) finished = true;
        }
        else
        {
          if (newI > exp->final_I) finished = true;
        }

        if (finished || exp->step_index >= exp->max_steps)
        {
          exp->inj->State = false;
          exp->state = DONE;
        }
        else
        {
          exp->inj->Imax = (float)newI;
          exp->pos = 0;
          exp->edge_count = 0;
          exp->last_Vd = exp->inj ? exp->inj->Vd : 0.0f;
          exp->last_Vq = exp->inj ? exp->inj->Vq : 0.0f;
          exp->state = INJECT_COLLECT;
        }
      }
      else
      {
        exp->state = DONE;
      }
      break;
    }

    case DONE:
      // nothing to do
      break;
  }
}

/* ---------- 读取结果数与单步结果 ---------- */
int Experiment_GetResultCount(FluxExperiment_t* exp)
{
  if (!exp) return 0;
  return exp->step_index;
}
const ImaxResult_t* Experiment_GetResult(FluxExperiment_t* exp, int idx)
{
  if (!exp) return NULL;
  if (idx < 0 || idx >= exp->step_index) return NULL;
  return &exp->results[idx];
}

// /* ----------------- 使用说明（示例流程） -----------------
// 1) 在系统初始化：
//     VoltageInjector_t inj = {0};
//     FluxExperiment_t exp;
//     Experiment_Init(&exp, &inj, Ts, SAMPLE_CAPACITY, CAPTURE_CYCLES, SELECT_CYCLES, MAX_STEPS,
//                     start_I, final_I, -1 /* step_dir -1 为从 start 向 min 递减 */, inject_amp,
//                     true /*use inj for Rs?*/);

// 2) 启动实验：
//     Experiment_Start(&exp);

// 3) 在每次采样周期（比如在 PWM 中断或定时器中）：
//     // 先调用估 Rs / 注入函数
//     // a) 如果你希望 Estimate_Rs 的 Voltage_out 直接通过 inj 发到电机，需要在 EST_RS 阶段让
//     SquareWaveGenerater 输出（已由 code 做部分处理）
//     // b) 在注入阶段，调用 SquareWaveGenerater 生成 Ud/Uq
//     float Ud, Uq;
//     SquareWaveGenerater(&inj, Id_meas, Iq_meas, &Ud, &Uq);
//     // 然后把测得的 Id/Iq 与 Ud/Uq 传入 Experiment_Step
//     Experiment_Step(&exp, Ud, Id_meas, Uq, Iq_meas, inj.Count);

// 4) 主循环或外部任务可轮询 Experiment_GetResultCount() 当状态为 DONE 时读取全部结果：
//     int n = Experiment_GetResultCount(&exp);
//     for (i=0;i<n;i++){
//         const ImaxResult_t *r = Experiment_GetResult(&exp,i);
//         // r->Imax_value, r->avg_max_psi
//     }
// ----------------------------------------------------------------- */

/* 注意与扩展
 - 当前实现以 d 轴为例计算 psi（psi_d_buf）。若要支持 q 轴（或同时支持），把处理段改成分别对
 psi_d_buf 与 psi_q_buf 计算最大值并保存。

 - 为保证实时性，建议把 Experiment_Step 的 PROCESS 阶段在主循环中运行（即在中断中只 push sample
 并在主循环检测到 state == PROCESS 再做 heavy calc）。当前实现把 PROCESS 放在 Experiment_Step
 里（同步执行），若你的中断时间有限，请拆分。
 - 若需要保存每个周期完整 (I,psi) 曲线以便离线 LLS，请告知，我会给出压缩保存版本（仍然静态分配）。
*/
bool Estimate_Rs(float Current, float* Voltage_out, float* Rs)
{
  static float V_last = 0.0F, I_last = 0.0F;
  static float V_now = 1.0F;
  static float Rs_last = 0.0F;
  static float Rs_new = 0.0F;
  static uint16_t hold = 0;
  static uint8_t est_step = 0;
  static bool first = true;
  static uint8_t done_flag = 0;
  static float I_filtered = 0.0F;  // 新增：滤波后的电流 //
  I_filtered = CURRENT_FILTER_ALPHA * Current +
               (1.0F - CURRENT_FILTER_ALPHA) * I_filtered;  // 如果已完成，直接输出V=0并返回true
  if (done_flag)
  {
    V_now = 0.0F;
    *Voltage_out = V_now;
    *Rs = Rs_new;
    return true;
  }
  *Voltage_out = V_now;
  if (hold < HOLD_CYCLES)
  {
    hold++;
  }
  else
  {
    hold = 0;
    if (first)
    {
      V_last = V_now;
      I_last = I_filtered;  // 用滤波后的电流 V_now += 1.0f;
      first = false;
      V_now += VOLTAGE_STEP;
    }
    else
    {
      float delta_I = I_filtered - I_last;
      if (fabsf(delta_I) < 1e-4f)
      {
        first = true;
        est_step = 0;
      }
      else
      {
        Rs_new = (V_now - V_last) / delta_I;
        float I_predict = I_last + (VOLTAGE_STEP / Rs_new);
        if ((fabsf(Rs_new - Rs_last) < RS_THRESHOLD && est_step > 0) &&
            (fabsf(I_filtered) >= CURRENT_LIMIT * CURRENT_RATIO))
        {
          *Rs = Rs_new;
          first = true;
          est_step = 0;
          done_flag = 1;
        }
        else if (fabsf(I_filtered) > CURRENT_LIMIT)
        {
          *Rs = Rs_new;
          first = true;
          est_step = 0;
          done_flag = 2;
          *Voltage_out = 0.0F;
        }
        else if (++est_step > MAX_STEPS)
        {
          *Rs = Rs_new;
          first = true;
          est_step = 0;
          done_flag = 4;
          *Voltage_out = 0.0F;
        }
        else
        {  // 只有预计电流不会超限时才步进
          if (fabsf(I_predict) < CURRENT_LIMIT * CURRENT_RATIO)
          {
            V_last = V_now;
            I_last = I_filtered;
            Rs_last = Rs_new;
            V_now += VOLTAGE_STEP;
            *Voltage_out = V_now;
          }
        }
      }
    }
  }
  return done_flag;
}
