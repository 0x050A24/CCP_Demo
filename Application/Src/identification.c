#include "identification.h"
#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "stdint.h"
#include "MTPA.h"
/* Estimate_Rs 与 SquareWaveGenerater 原型（你已有的实现） */
static inline bool Estimate_Rs(float Current, float* Voltage_out, float* Rs);
static inline void square_wave_injector(FluxExperiment_t* exp, float Id, float Iq, float* Ud,
                                        float* Uq);
static bool edge_detect(FluxExperiment_t* exp, float* Ud, float* Uq);
static inline void cycle_capture(FluxExperiment_t* exp);
static inline void buffer_load(FluxExperiment_t* exp, float Id, float Iq);
static inline bool cal_single_psi(FluxExperiment_t* exp);
static inline bool cal_avg_max_psi(FluxExperiment_t* exp);
static inline bool cal_DQ_psi(FluxExperiment_t* exp);
static inline bool cal_next_current(FluxExperiment_t* exp);
static inline void Single_Axis_LLS(FluxExperiment_t* exp, int exponent, Result_t* result);
static inline void Cross_Axis_LLS(FluxExperiment_t* exp, ResultDQ_t* result);
static void process_cycle_for_dq_adq(FluxExperiment_t* exp, int s);

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
  // 初始化方波注入器
  exp->inj.Ud_amp = 0.0F;
  exp->inj.Uq_amp = 0.0F;
  exp->inj.Imax = 0.0F;
  exp->inj.State = false;
  exp->inj.mode = INJECT_D;
  exp->inj.inj_state_d = 0;
  exp->inj.inj_state_q = 0;
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
      exp->Running = true;
      float Voltage_out = 0.0f;
      float Rs_tmp = 0.0f;

      // 直接把 Rs 估计电压输出到电机
      *Ud = Voltage_out;
      *Uq = 0.0f;

      if (Estimate_Rs(Id, &Voltage_out, &Rs_tmp))
      {
        // 保存 Rs
        exp->Rs_est = Rs_tmp;

        // 关闭电压输出
        *Ud = 0.0f;
        *Uq = 0.0f;

        // exp->state = INJECT_COLLECT;
        exp->state = WAIT;
      }
      break;
    }

    case INJECT_COLLECT:
    {
      // --- 更新注入器状态 ---
      square_wave_injector(exp, Id, Iq, Ud, Uq);

      if (edge_detect(exp, Ud, Uq))
      {
        cycle_capture(exp);
      }

      // --- 存 buffer（仅在等待期结束后才写入） ---
      if (exp->edge_count >= exp->wait_edges + 1)
      {
        if (exp->pos < exp->sample_capacity)
        {
          buffer_load(exp, Id, Iq);
        }
        else
        {
          // buffer 满，进入处理
          exp->state = PROCESS;
          exp->inj.State = false;
          exp->inj.Vd = 0.0F;
          exp->inj.Vq = 0.0F;
        }
      }
      exp->last_Vd = *Ud;
      exp->last_Vq = *Uq;
      break;
    }

    case PROCESS:
    {
      exp->inj.State = false;
      *Ud = exp->inj.Vd = 0.0F;
      *Uq = exp->inj.Vq = 0.0F;
      if (exp->edge_count < 2)
      {
        // 不应发生（INJECT_COLLECT 已经保证 >= wait_edges+3 才进入 PROCESS）
        exp->state = NEXT_I;
        break;
      }
      if (exp->inj.mode == INJECT_D || exp->inj.mode == INJECT_Q)
      {
        if (cal_single_psi(exp) == false)
        {
          // 无效数据，直接重试
          exp->inj.State = true;
          exp->state = INJECT_COLLECT;
          break;
        }

        if (cal_avg_max_psi(exp))
        {
          exp->state = NEXT_I;
        }
        else
        {
          // 未达到重复次数
          exp->inj.State = true;  // 重新开启注入
          exp->state = INJECT_COLLECT;
        }
      }

      if (exp->inj.mode == INJECT_DQ)
      {
        if (cal_DQ_psi(exp) == false)
        {
          // 无效数据，直接重试
          exp->inj.State = true;
          exp->state = INJECT_COLLECT;
        }
        else
        {
          exp->inj.State = false;
          exp->state = LLS;
        }
      }
      break;
    }
    case NEXT_I:
    {
      if (cal_next_current(exp))
      {
        exp->inj.State = false;
        exp->state = LLS;
      }
      else
      {
        exp->state = INJECT_COLLECT;
        // exp->inj.State = true;
      }
      break;
    }

    case LLS:
    {
      uint8_t X = 0;
      if (exp->inj.mode == INJECT_D)
      {
        X = S;
        Single_Axis_LLS(exp, X, &exp->LLS.D);  // X=5
      }
      if (exp->inj.mode == INJECT_Q)
      {
        X = Tn;
        Single_Axis_LLS(exp, X, &exp->LLS.Q);  // X=1
      }
      if (exp->inj.mode == INJECT_DQ)
      {
        Cross_Axis_LLS(exp, &exp->LLS.DQ);  // (S=5,T=1,U=1,V=0)
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
        exp->state = PROCESS;
      }
      else if (exp->inj.mode == INJECT_DQ)
      {
        exp->inj.mode = INJECT_D;
        exp->state = DONE;
      }
      break;
    }
    case DONE:
    {
      *Ud = 0.0f;
      *Uq = 0.0f;
    assign_parameters_from_LLS(exp->LLS);

      exp->Running = false;
      break;
    }
  }
}

static bool Estimate_Rs(float Current, float* Voltage_out, float* Rs)
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

void Single_Axis_LLS(FluxExperiment_t* exp, int exponent, Result_t* result)
{
  float sum_x2 = 0.0f;
  float sum_xp = 0.0f;
  float sum_xp2 = 0.0f;
  float sum_yx = 0.0f;
  float sum_yxp = 0.0f;
  float sum_y = 0.0f;
  // float sum_y2 = 0.0f;
  int N = exp->step_index;

  for (int i = 0; i < N; i++)
  {
    float psi = exp->results[i].avg_max_psi;
    float I = exp->results[i].Imax_value;

    // 幂次计算：psi^(exponent+1)
    float xp = 1.0f;
    for (int k = 0; k < exponent + 1; k++)
    {
      xp *= psi;
    }

    sum_x2 += psi * psi;
    sum_xp += psi * xp;
    sum_xp2 += xp * xp;

    sum_y += I;
    // sum_y2 += I * I;

    sum_yx += psi * I;
    sum_yxp += xp * I;
  }

  float det = sum_x2 * sum_xp2 - sum_xp * sum_xp;

  // --- 计算残差平方和 J 和 R² ---
  float ss_res = 0.0f;  // 残差平方和
  float ss_tot = 0.0f;  // 总平方和
  float mean_y = sum_y / (float)N;

  if (fabsf(det) > 1e-12f)
  {
    float b0 = 0.0f, b1 = 0.0f;

    result->beta0 = b0 = (sum_xp2 * sum_yx - sum_xp * sum_yxp) / det;
    result->beta1 = b1 = (sum_x2 * sum_yxp - sum_xp * sum_yx) / det;
    for (int i = 0; i < N; i++)
    {
      float psi = exp->results[i].avg_max_psi;
      float I = exp->results[i].Imax_value;

      float xp = 1.0f;
      for (int k = 0; k < exponent + 1; k++)
      {
        xp *= psi;
      }

      float I_hat = b0 * psi + b1 * xp;
      float err = I - I_hat;

      ss_res += err * err;
      float diff = I - mean_y;
      ss_tot += diff * diff;
    }

    result->J = ss_res;
    result->R2 = (ss_tot > 1e-12f) ? (1.0f - ss_res / ss_tot) : 0.0f;
  }
}

bool edge_detect(FluxExperiment_t* exp, float* Ud, float* Uq)
{
  bool edge_detected = false;

  if (exp->inj.mode == INJECT_D || exp->inj.mode == INJECT_DQ)
  {
    if (*Ud == -exp->last_Vd) edge_detected = true;
  }
  if (exp->inj.mode == INJECT_Q)
  {
    if (*Uq == -exp->last_Vq) edge_detected = true;
  }

  return edge_detected;
}

void cycle_capture(FluxExperiment_t* exp)
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
  }
}

void buffer_load(FluxExperiment_t* exp, float Id, float Iq)
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

  if (exp->inj.mode == INJECT_DQ)
  {
    exp->repeat_count++;
  }
}

void square_wave_injector(FluxExperiment_t* exp, float Id, float Iq, float* Ud, float* Uq)
{
  if (exp->inj.State == false)
  {
    exp->inj.Vd = 0.0F;
    exp->inj.Vq = 0.0F;
    *Ud = exp->inj.Vd;
    *Uq = exp->inj.Vq;
    return;
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
}

bool cal_single_psi(FluxExperiment_t* exp)
{
  bool D_Q_calculated = false;
  int s_idx = exp->edge_idx[0];
  int e_idx = exp->edge_idx[1];
  // 检查样本数是否足够
  if (s_idx <= e_idx + 1)
  {
    // 本次周期数据不足，重做一次采集（不计入 repeat_count）
    exp->pos = 0;
    exp->edge_count = 0;
    // 重新开启注入以重试
    exp->last_Vd = 0.0f;
    exp->last_Vq = 0.0f;
    D_Q_calculated = false;
  }
  else
  {
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
    float I_at_max = 0.0f;

    for (int i = s_idx; i < e_idx; ++i)
    {
      float psi_c = psi_buf[i] - mean;
      if (psi_c > max_psi)
      {
        max_psi = psi_c;
        // 取磁链峰值点对应的电流
        if (exp->inj.mode == INJECT_D)
        {
          I_at_max = exp->Id_buf[i];
        }
        else if (exp->inj.mode == INJECT_Q)
        {
          I_at_max = exp->Iq_buf[i];
        }
      }

      // 如果数据有效，累积；否则重试（不计入）
      if (max_psi > -1e29f)
      {
        exp->sum_max_psi += max_psi;
        exp->sum_max_I += I_at_max;  // 新增
        exp->repeat_count++;
        D_Q_calculated = true;
      }
      else
      {
        D_Q_calculated = false;
      }
    }
  }
  return D_Q_calculated;
}

bool cal_avg_max_psi(FluxExperiment_t* exp)
{
  bool calculated = false;
  // ---- 判断是否已经达到重复次数 ----
  if (exp->repeat_count < exp->repeat_times)
  {
    // 还需重复：为下一次注入做准备
    calculated = false;
    exp->pos = 0;
    exp->edge_count = 0;
  }
  else
  {
    // 达到重复次数：计算平均并保存结果
    float avg_psi = exp->sum_max_psi / (float)exp->repeat_times;
    float avg_I = exp->sum_max_I / (float)exp->repeat_times;

    exp->results[exp->step_index].Imax_value = avg_I;     // 现在是峰值点对应的电流
    exp->results[exp->step_index].avg_max_psi = avg_psi;  // 峰值点磁链
    exp->results[exp->step_index].cycles_used = exp->repeat_times;
    // 临时
    save_result(avg_I, avg_psi, exp->repeat_times);

    exp->step_index++;

    // 清零累积器，为下一 Imax 做准备（NEXT_I 也会重置 pos/edge_count）
    exp->sum_max_psi = 0.0F;
    exp->sum_max_I = 0.0F;
    exp->repeat_count = 0;
    calculated = true;
  }
  return calculated;
}

static inline bool cal_DQ_psi(FluxExperiment_t* exp)
{
  bool calculated = false;
  // DQ 轴同时计算
  float* psi_buf_d = exp->psi_d_buf;
  float* psi_buf_q = exp->psi_q_buf;

  // 计算 d 轴均值
  float sum_d = 0.0f;
  int cnt_d = 0;
  for (int i = exp->edge_idx[0]; i < exp->edge_idx[1]; ++i)
  {
    sum_d += psi_buf_d[i];
    cnt_d++;
  }
  float mean_d = (cnt_d > 0) ? (sum_d / (float)cnt_d) : 0.0f;

  // 计算 q 轴均值
  float sum_q = 0.0f;
  int cnt_q = 0;
  for (int i = exp->edge_idx[0]; i < exp->edge_idx[1]; ++i)
  {
    sum_q += psi_buf_q[i];
    cnt_q++;
  }
  float mean_q = (cnt_q > 0) ? (sum_q / (float)cnt_q) : 0.0f;

  for (int i = exp->edge_idx[0]; i < exp->edge_idx[1]; ++i)
  {
    psi_buf_d[i] -= mean_d;

    psi_buf_q[i] -= mean_q;
  }
  process_cycle_for_dq_adq(exp, S);
  // ---- 判断是否已经达到重复次数 ----
  if (exp->repeat_count < exp->repeat_times)
  {
    // 还需重复：为下一次注入做准备
    exp->pos = 0;
    exp->edge_count = 0;
    calculated = false;
  }
  else
  {
    calculated = true;
  }
  return calculated;
}

bool cal_next_current(FluxExperiment_t* exp)
{
  bool isenough = false;
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
    isenough = true;
  }
  else
  {
    isenough = false;
    exp->inj.Imax = (float)newI;
    exp->pos = 0;
    exp->edge_count = 0;
    exp->state = INJECT_COLLECT;
    // exp->inj.State = true;
  }
  return isenough;
}

void process_cycle_for_dq_adq(FluxExperiment_t* exp, int s)
{
  for (int i = exp->edge_idx[0]; i < exp->edge_idx[1]; ++i)
  {
    float id_s = exp->Id_buf[i];
    float iq_s = exp->Iq_buf[i];
    float psi_d = exp->psi_d_buf[i];
    float psi_q = exp->psi_q_buf[i];

    // only positive quadrant
    if (!(id_s > 0.0F && iq_s > 0.0F && psi_d > 0.0F && psi_q > 0.0F)) continue;

    // compute psi^powers efficiently
    float psi_d_S1 = 1.0F;
    for (int k = 0; k < s + 1; ++k) psi_d_S1 *= psi_d;  // psi_d^(S+1)
    float psi_q_T1 = 1.0F;
    for (int k = 0; k < Tn + 1; ++k) psi_q_T1 *= psi_q;  // psi_q^(T+1)

    float id_pred = exp->LLS.D.beta0 * psi_d + exp->LLS.D.beta1 * psi_d_S1;
    float iq_pred = exp->LLS.Q.beta0 * psi_q + exp->LLS.Q.beta1 * psi_q_T1;

    float id_res = id_s - id_pred;
    float iq_res = iq_s - iq_pred;

    // x1 and x2
    float psi_d_U1 = 1.0F;
    for (int k = 0; k < U + 1; ++k) psi_d_U1 *= psi_d;  // psi_d^(U+1)
    float psi_d_U2 = psi_d_U1 * psi_d;                  // psi_d^(U+2)
    float psi_q_V1 = 1.0F;
    for (int k = 0; k < V + 1; ++k) psi_q_V1 *= psi_q;  // psi_q^(V+1)
    float psi_q_V2 = psi_q_V1 * psi_q;                  // psi_q^(V+2)

    float x1 = (psi_d_U1 * psi_q_V2) / (float)(V + 2);
    float x2 = (psi_d_U2 * psi_q_V1) / (float)(U + 2);

    // accumulate Sxx, Sxy
    exp->cq_Sxx += x1 * x1 + x2 * x2;
    exp->cq_Sxy += x1 * id_res + x2 * iq_res;

    // accumulate sums for R2
    exp->sum_id += id_s;
    exp->sum_id2 += id_s * id_s;
    exp->count_id++;
    exp->sum_iq += iq_s;
    exp->sum_iq2 += iq_s * iq_s;
    exp->count_iq++;

    // residual sums
    exp->sum_eps_id2 += id_res * id_res;
    exp->sum_eps_iq2 += iq_res * iq_res;
  }
}

static inline void Cross_Axis_LLS(FluxExperiment_t* exp, ResultDQ_t* result)
{
  float adq = 0.0f;
  if (exp->cq_Sxx > 1e-12f)
  {
    adq = exp->cq_Sxy / exp->cq_Sxx;
  }
  else
  {
    adq = -1.0f;  // 或标记失败 / 正则化
  }

  // J (残差平方和)
  float Jd = exp->sum_eps_id2;
  float Jq = exp->sum_eps_iq2;

  // 计算 R^2

  float SST_id =
      exp->sum_id2 - (exp->sum_id * exp->sum_id) / (float)exp->count_id;  // sum (id - mean)^2
  float R2_d = (SST_id > 0.0f) ? (1.0f - Jd / SST_id) : 0.0f;

  float SST_iq = exp->sum_iq2 - (exp->sum_iq * exp->sum_iq) / (float)exp->count_iq;
  float R2_q = (SST_iq > 0.0f) ? (1.0f - Jq / SST_iq) : 0.0f;

  result->adq = adq;
  result->J[0] = Jd;
  result->J[1] = Jq;
  result->R2[0] = R2_d;
  result->R2[1] = R2_q;
}
void assign_parameters_from_LLS(LLS_Result_t res)
{
    a_d = res.D.beta0; b_d = res.D.beta1;
    a_q = res.Q.beta0; b_q = res.Q.beta1;
    c_coeff = res.DQ.adq; 
}