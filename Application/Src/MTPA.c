#include "MTPA.h"
#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "stdint.h"

/* Estimate_Rs 与 SquareWaveGenerater 原型（你已有的实现） */
static inline bool Estimate_Rs(float Current, float* Voltage_out, float* Rs);
static LLS_Result_t Single_Axis_LLS(FluxExperiment_t* exp, int exponent);

/* 结果结构：每个 Imax 只保存最终的 avg_max_psi（和 Imax 值） */

volatile float g_results_Imax[MAX_STEPS];
volatile float g_results_avgmax[MAX_STEPS];
volatile float g_results_ad0;
volatile float g_results_add;
volatile float g_results_aq0;
volatile float g_results_aqq;
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
void Experiment_Init(FluxExperiment_t* exp, float Ts, int sample_capacity, int repeat_times,
                     int max_steps, int start_I, int final_I, int step_dir, float inject_amp)
{
  // clip params
  if (sample_capacity > SAMPLE_CAPACITY) sample_capacity = SAMPLE_CAPACITY;
  if (max_steps > MAX_STEPS) max_steps = MAX_STEPS;
  if (repeat_times > REPEAT_TIMES) repeat_times = REPEAT_TIMES;

  memset(exp, 0, sizeof(FluxExperiment_t));
  exp->Ts = Ts;
  exp->sample_capacity = sample_capacity;
  exp->max_steps = max_steps;
  exp->repeat_times = repeat_times;
  exp->wait_edges = 3;
  exp->start_I = start_I;
  exp->final_I = final_I;
  exp->step_dir = (step_dir >= 0) ? 1 : -1;
  exp->inject_amp = inject_amp;
  exp->Running = false;
  exp->pos = 0;
  exp->edge_count = 0;
  exp->step_index = 0;
  exp->state = WAIT;
  exp->last_Vd = 0.0f;
  exp->last_Vq = 0.0f;
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
        exp->inj.Ud_amp = 0.0F;
        exp->inj.Uq_amp = 0.0F;
        exp->inj.Imax = 0.0F;
        exp->inj.State = false;
        exp->inj.mode = INJECT_D;
        exp->inj.inj_state_d = 0;
        exp->inj.inj_state_q = 0;

        exp->pos = 0;
        exp->edge_count = 0;
        exp->last_Vd = 0.0f;
        exp->last_Vq = 0.0f;
        // exp->state = INJECT_COLLECT;
        exp->state = WAIT;
      }
      break;
    }

    case INJECT_COLLECT:
    {
      if (exp->inj.State == false)
      {
        exp->inj.Vd = 0.0F;
        exp->inj.Vq = 0.0F;
        *Ud = exp->inj.Vd;
        *Uq = exp->inj.Vq;
        break;
      }

      if (exp->inj.State)
      {
        // --- 方波注入 D 轴 ---
        if (exp->inj.mode == INJECT_D || exp->inj.mode == INJECT_DQ)
        {
          if (Id >= exp->inj.Imax)
          {
            exp->inj.inj_state_d = -1;
          }
          else if (Id <= -exp->inj.Imax)
          {
            exp->inj.inj_state_d = +1;
          }
          exp->inj.Vd = (exp->inj.inj_state_d >= 0) ? exp->inj.Ud_amp : -exp->inj.Ud_amp;
        }

        // --- 方波注入 Q 轴 ---
        if (exp->inj.mode == INJECT_Q || exp->inj.mode == INJECT_DQ)
        {
          if (Iq >= exp->inj.Imax)
          {
            exp->inj.inj_state_q = -1;
          }
          else if (Iq <= -exp->inj.Imax)
          {
            exp->inj.inj_state_q = +1;
          }
          exp->inj.Vq = (exp->inj.inj_state_q >= 0) ? exp->inj.Uq_amp : -exp->inj.Uq_amp;
        }

        // 本次输出电压
        *Ud = exp->inj.Vd;
        *Uq = exp->inj.Vq;
      }

      bool edge_detected = false;

      if (exp->inj.mode == INJECT_D || exp->inj.mode == INJECT_DQ)
      {
        if (*Ud == -exp->last_Vd) edge_detected = true;
      }
      if (exp->inj.mode == INJECT_Q)
      {
        if (*Uq == -exp->last_Vq) edge_detected = true;
      }

      if (edge_detected)
      {
        exp->edge_count++;

        // 第一次有效边沿，记录起点
        if (exp->edge_count == exp->wait_edges + 1)
        {
          exp->pos = 0;  // buffer 从 0 开始存
          exp->edge_idx[0] = 0;
        }

        // 收到 wait_edges + 3 个边沿时，说明完整周期结束
        if (exp->edge_count == exp->wait_edges + 3)
        {
          exp->edge_idx[1] = exp->pos;
          exp->state = PROCESS;
          exp->inj.State = false;
          exp->inj.Vd = 0.0F;
          exp->inj.Vq = 0.0F;
          *Ud = exp->inj.Vd;
          *Uq = exp->inj.Vq;
        }
      }

      // --- 存 buffer（仅在等待期结束后才写入） ---
      if (exp->edge_count >= exp->wait_edges + 1)
      {
        if (exp->pos < exp->sample_capacity)
        {
          int idx = exp->pos;
          exp->Ud_buf[idx] = exp->last_Vd;
          exp->Id_buf[idx] = Id;
          exp->Uq_buf[idx] = exp->last_Vq;
          exp->Iq_buf[idx] = Iq;

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
          exp->inj.State = false;
          exp->inj.Vd = 0.0F;
          exp->inj.Vq = 0.0F;
          *Ud = exp->inj.Vd;
          *Uq = exp->inj.Vq;
        }
      }
      exp->last_Vd = *Ud;
      exp->last_Vq = *Uq;
      break;
    }

    case PROCESS:
    {
      // ---- 用 INJECT_COLLECT 写好的两个索引表示一个周期 ----
      // edge_idx[0] = 起点（写入 buffer 时的 0）
      // edge_idx[1] = 结束位置（写入时 pos）
      if (exp->edge_count < 2)
      {
        // 不应发生（INJECT_COLLECT 已经保证 >= wait_edges+3 才进入 PROCESS）
        exp->state = NEXT_I;
        break;
      }

      int s_idx = exp->edge_idx[0];
      int e_idx = exp->edge_idx[1];

      // 检查样本数是否足够
      if (e_idx <= s_idx + 1)
      {
        // 本次周期数据不足，重做一次采集（不计入 repeat_count）
        exp->pos = 0;
        exp->edge_count = 0;
        // 重新开启注入以重试
        exp->inj.State = true;
        exp->last_Vd = 0.0f;
        exp->last_Vq = 0.0f;
        exp->state = INJECT_COLLECT;
        break;
      }

      // 选择要用的 psi 缓冲区：Q 注入用 psi_q，否则使用 psi_d（如果需要同时计算可再扩展）
      float* psi_buf = (exp->inj.mode == INJECT_Q) ? exp->psi_q_buf : exp->psi_d_buf;

      // ---- 计算去均值后的最大 psi ----
      float sum = 0.0f;
      int cnt = 0;
      for (int i = s_idx; i < e_idx; ++i)
      {
        sum += psi_buf[i];
        cnt++;
      }
      float mean = (cnt > 0) ? (sum / (float)cnt) : 0.0f;

      float max_psi = -1e30f;
      for (int i = s_idx; i < e_idx; ++i)
      {
        float psi_c = psi_buf[i] - mean;
        if (psi_c > max_psi) max_psi = psi_c;
      }

      // 如果数据有效，累积；否则重试（不计入）
      if (max_psi > -1e29f)
      {
        exp->sum_max_psi += max_psi;
        exp->repeat_count++;
      }
      else
      {
        // 无效数据，直接重试
        exp->pos = 0;
        exp->edge_count = 0;
        exp->inj.State = true;
        exp->state = INJECT_COLLECT;
        break;
      }

      // ---- 判断是否已经达到重复次数 ----
      if (exp->repeat_count < exp->repeat_times)
      {
        // 还需重复：为下一次注入做准备
        exp->pos = 0;
        exp->edge_count = 0;
        exp->inj.State = true;  // 重新开启注入
        exp->state = INJECT_COLLECT;
      }
      else
      {
        // 达到重复次数：计算平均并保存结果
        float avg = exp->sum_max_psi / (float)exp->repeat_times;

        exp->results[exp->step_index].Imax_value = exp->inj.Imax;
        exp->results[exp->step_index].avg_max_psi = avg;
        exp->results[exp->step_index].cycles_used = exp->repeat_times;
        // 若你同时使用 save_result，也可以调用：
        save_result(exp->inj.Imax, avg, exp->repeat_times);

        exp->step_index++;

        // 清零累积器，为下一 Imax 做准备（NEXT_I 也会重置 pos/edge_count）
        exp->sum_max_psi = 0.0f;
        exp->repeat_count = 0;

        exp->state = NEXT_I;
      }
      break;
    }

    case NEXT_I:
    {
      int curI = (int)exp->inj.Imax;
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
        exp->inj.State = false;
        exp->state = LLS;
      }
      else
      {
        exp->inj.Imax = (float)newI;
        exp->pos = 0;
        exp->edge_count = 0;
        exp->state = INJECT_COLLECT;
      }
      break;
    }

    case LLS:
    {
      uint8_t X = 0;
      LLS_Result_t lls;
      if (exp->inj.mode == INJECT_D)
      {
        X = 5;
        lls = Single_Axis_LLS(exp, X);  // X=5
        g_results_ad0 = lls.ad0;
        g_results_add = lls.add;
      }
      if (exp->inj.mode == INJECT_Q)
      {
        X = 1;
        lls = Single_Axis_LLS(exp, X);  // X=1
        g_results_aq0 = lls.aq0;
        g_results_aqq = lls.aqq;
      }

      exp->state = PENDING;
      break;
    }
    case PENDING:
    {
      // 初始化方波注入器
      exp->inj.Ud_amp = 0.0F;
      exp->inj.Uq_amp = 0.0F;
      exp->inj.Imax = 0.0F;
      exp->inj.State = false;
      exp->inj.inj_state_d = 0;
      exp->inj.inj_state_q = 0;
      exp->pos = 0;
      exp->edge_count = 0;
      exp->last_Vd = 0.0f;
      exp->last_Vq = 0.0f;
      exp->step_index = 0;
      if (exp->inj.mode == INJECT_D)
      {
        exp->inj.mode = INJECT_Q;
        exp->state = PROCESS;
      }
      else if (exp->inj.mode == INJECT_Q)
      {
        exp->inj.mode = INJECT_DQ;
        exp->state = DONE;
        // exp->state = PROCESS;
      }
      // else if (exp->inj.mode == INJECT_DQ)
      // {
      //   exp->inj.mode = INJECT_D;
      //   exp->state = DONE;
      // }
      break;
    }
    case DONE:
    {
      *Ud = 0.0f;
      *Uq = 0.0f;
      exp->Running = false;
      break;
    }
  }
}

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

LLS_Result_t Single_Axis_LLS(FluxExperiment_t* exp, int exponent)
{
  float sum_x2 = 0.0f;
  float sum_xp = 0.0f;
  float sum_xp2 = 0.0f;
  float sum_yx = 0.0f;
  float sum_yxp = 0.0f;
  int N = exp->step_index;

  for (int i = 0; i < N; i++)
  {
    float psi = exp->results[i].avg_max_psi;
    float I = exp->results[i].Imax_value;

    // 幂次计算：psi^(S+1)
    float xp = 1.0f;
    for (int k = 0; k < exponent + 1; k++)
    {
      xp *= psi;
    }

    sum_x2 += psi * psi;
    sum_xp += psi * xp;
    sum_xp2 += xp * xp;

    sum_yx += psi * I;
    sum_yxp += xp * I;
  }

  float det = sum_x2 * sum_xp2 - sum_xp * sum_xp;
  LLS_Result_t res = {0};

  if (fabsf(det) > 1e-12f)
  {
    if (exp->inj.mode == INJECT_D)
    {
      res.ad0 = (sum_xp2 * sum_yx - sum_xp * sum_yxp) / det;
      res.add = (sum_x2 * sum_yxp - sum_xp * sum_yx) / det;
    }
    else if (exp->inj.mode == INJECT_Q)
    {
      res.aq0 = (sum_xp2 * sum_yx - sum_xp * sum_yxp) / det;
      res.aqq = (sum_x2 * sum_yxp - sum_xp * sum_yx) / det;
    }
  }

  return res;
}
