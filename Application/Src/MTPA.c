#include "MTPA.h"
#include <stdbool.h>
#include <string.h>
#include "math.h"
#include "stdint.h"

/* Estimate_Rs 与 SquareWaveGenerater 原型（你已有的实现） */
static inline bool Estimate_Rs(float Current, float* Voltage_out, float* Rs);
static void square_injection(FluxExperiment_t* exp, float Id, float Iq, float* Ud, float* Uq);
static bool detect_edge(FluxExperiment_t* exp, float Ud, float Uq);
static void collect_sample(FluxExperiment_t* exp, float Id, float Iq);
static bool process_single_axis_cycle(FluxExperiment_t* exp, float* psi_buf, float* I_buf,
                                      float* avg_I, float* avg_psi);
static bool next_imax(FluxExperiment_t* exp);
static LLS_Result_t Single_Axis_LLS(FluxExperiment_t* exp, int exponent);
static void process_cycle_for_dq_adq(FluxExperiment_t* exp, int s);

/* 结果结构：每个 Imax 只保存最终的 avg_max_psi（和 Imax 值） */

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

    case D_AXIS_INJECT:
    {
      // 如果延时未激活，初始化注入
      if (!exp->inj.State && !exp->timer_active)
      {
        exp->inj.State = true;
        exp->inj.Ud_amp = exp->inject_amp;
        exp->inj.Uq_amp = 0.0f;
        if (exp->inj.Imax == 0.0f) next_imax(exp);  // 首次设置 Imax
      }

      // 注入状态
      if (exp->inj.State)
      {
        square_injection(exp, Id, Iq, Ud, Uq);
        bool full_cycle = detect_edge(exp, *Ud, *Uq);

        // 数据采样
        if (exp->edge_count >= exp->wait_edges + 1)
        {
          collect_sample(exp, Id, Iq);
          if (exp->pos >= exp->sample_capacity)
          {
            exp->inj.State = false;  // 停止注入
            *Ud = 0.0f;
            *Uq = 0.0f;
          }
        }

        // 完整周期检测
        if (full_cycle)
        {
          exp->inj.State = false;  // 注入结束，进入数据处理
          *Ud = 0.0f;
          *Uq = 0.0f;
          exp->state = D_AXIS_PROCESS;
        }
      }

      break;
    }

    case D_AXIS_PROCESS:
    {
      float avg_I = 0.0f, avg_psi = 0.0f;
      bool cycle_done =
          process_single_axis_cycle(exp, exp->psi_d_buf, exp->Id_buf, &avg_I, &avg_psi);

      if (cycle_done)
      {
        // 保存结果
        exp->results[exp->step_index].avg_max_psi = avg_psi;
        exp->results[exp->step_index].Imax_value = avg_I;
        exp->results[exp->step_index].cycles_used = exp->repeat_times;

        // step_index 自增
        exp->step_index++;

        // 启动延时
        exp->timer_active = true;
        exp->wait_timer = 0.0f;

        // 关闭注入
        exp->inj.State = false;
        *Ud = 0.0f;
        *Uq = 0.0f;
      }
      else
      {
        // repeat 未完成 → 重试
        exp->pos = 0;
        exp->edge_count = 0;
        exp->inj.State = true;
        exp->state = D_AXIS_INJECT;
      }

      // 延时期间
      if (exp->timer_active)
      {
        exp->wait_timer += exp->Ts;
        *Ud = 0.0f;
        *Uq = 0.0f;
        exp->inj.State = false;

        if (exp->wait_timer >= 1.0f)
        {
          exp->timer_active = false;
          exp->wait_timer = 0.0f;

          // 切换 Imax
          if (!next_imax(exp))
          {
            Single_Axis_LLS(exp, 5);
            exp->state = DONE;
          }
          else
          {
            // 下一轮采样
            exp->pos = 0;
            exp->edge_count = 0;
            exp->repeat_count = 0;
            exp->inj.State = true;
            exp->state = D_AXIS_INJECT;
          }
        }
      }

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

LLS_Result_t Single_Axis_LLS(FluxExperiment_t* exp, int exponent)
{
  float sum_x2 = 0.0f;
  float sum_xp = 0.0f;
  float sum_xp2 = 0.0f;
  float sum_yx = 0.0f;
  float sum_yxp = 0.0f;
  float sum_y = 0.0f;
  float sum_y2 = 0.0f;
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
    sum_y2 += I * I;

    sum_yx += psi * I;
    sum_yxp += xp * I;
  }

  float det = sum_x2 * sum_xp2 - sum_xp * sum_xp;

  // --- 计算残差平方和 J 和 R² ---
  float ss_res = 0.0f;  // 残差平方和
  float ss_tot = 0.0f;  // 总平方和
  float mean_y = sum_y / (float)N;
  LLS_Result_t res = {0};

  if (fabsf(det) > 1e-12f)
  {
    float b0 = 0.0f, b1 = 0.0f;

    if (exp->inj.mode == INJECT_D)
    {
      res.ad0 = b0 = (sum_xp2 * sum_yx - sum_xp * sum_yxp) / det;
      res.add = b1 = (sum_x2 * sum_yxp - sum_xp * sum_yx) / det;
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

      res.D.J = ss_res;
      res.D.R2 = (ss_tot > 1e-12f) ? (1.0f - ss_res / ss_tot) : 0.0f;
    }
    else if (exp->inj.mode == INJECT_Q)
    {
      res.aq0 = b0 = (sum_xp2 * sum_yx - sum_xp * sum_yxp) / det;
      res.aqq = b1 = (sum_x2 * sum_yxp - sum_xp * sum_yx) / det;
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

      res.Q.J = ss_res;
      res.Q.R2 = (ss_tot > 1e-12f) ? (1.0f - ss_res / ss_tot) : 0.0f;
    }
  }

  // 存入 exp->LLS，方便外部调用
  exp->LLS = res;

  return res;
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
    for (int k = 0; k < T + 1; ++k) psi_q_T1 *= psi_q;  // psi_q^(T+1)

    float id_pred = exp->LLS.ad0 * psi_d + exp->LLS.add * psi_d_S1;
    float iq_pred = exp->LLS.aq0 * psi_q + exp->LLS.aqq * psi_q_T1;

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

// 方波注入电压（D/Q/DQ 通用）
static void square_injection(FluxExperiment_t* exp, float Id, float Iq, float* Ud, float* Uq)
{
  if (!exp->inj.State)
  {
    // 注入关闭：输出 0，不改变 inj_state 以保持相位连续性
    exp->inj.Vd = 0.0f;
    exp->inj.Vq = 0.0f;
    *Ud = 0.0f;
    *Uq = 0.0f;
    return;
  }

  // D 轴
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

  // Q 轴
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

  *Ud = exp->inj.Vd;
  *Uq = exp->inj.Vq;
}

// 返回 true 表示检测到完整周期（wait_edges + 3 个有效边沿）
// 注意：本函数只计数，不做 pos/edge_count 的清零（上层负责）
static bool detect_edge(FluxExperiment_t* exp, float Ud, float Uq)
{
  if (!exp->inj.State)
  {
    // 注入未开启，不计边沿
    exp->last_Vd = Ud;
    exp->last_Vq = Uq;
    return false;
  }

  bool edge_detected = false;

  if (exp->inj.mode == INJECT_D || exp->inj.mode == INJECT_DQ)
  {
    if (Ud == -exp->last_Vd) edge_detected = true;
  }
  else if (exp->inj.mode == INJECT_Q)
  {
    if (Uq == -exp->last_Vq) edge_detected = true;
  }

  if (edge_detected)
  {
    exp->edge_count++;

    // 第一次有效边沿，记录起点（保留原逻辑）
    if (exp->edge_count == exp->wait_edges + 1)
    {
      exp->pos = 0;  // buffer 从 0 开始存
      exp->edge_idx[0] = 0;
    }

    // 收到 wait_edges + 3 个边沿时，说明完整周期结束
    if (exp->edge_count == exp->wait_edges + 3)
    {
      exp->edge_idx[1] = exp->pos;
      // 注意：不在这里清零 edge_count/pos — 上层处理完后负责重置或续采
      exp->last_Vd = Ud;
      exp->last_Vq = Uq;
      return true;
    }
  }

  exp->last_Vd = Ud;
  exp->last_Vq = Uq;
  return false;
}

// 缓冲区写入 + 磁链积分
static void collect_sample(FluxExperiment_t* exp, float Id, float Iq)
{
  if (exp->pos >= exp->sample_capacity) return;

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
    float integrand_d = exp->last_Vd - exp->Rs_est * Id;
    float integrand_q = exp->last_Vq - exp->Rs_est * Iq;
    exp->psi_d_buf[idx] = exp->psi_d_buf[prev] + exp->Ts * integrand_d;
    exp->psi_q_buf[idx] = exp->psi_q_buf[prev] + exp->Ts * integrand_q;
  }
  exp->pos++;
}

// 计算均值并找到峰值点（D/Q 单轴模式用）
static bool process_single_axis_cycle(FluxExperiment_t* exp, float* psi_buf, float* I_buf,
                                      float* avg_I, float* avg_psi)
{
  float sum = 0.0f;
  int cnt = 0;
  for (int i = exp->edge_idx[0]; i < exp->edge_idx[1]; i++)
  {
    sum += psi_buf[i];
    cnt++;
  }
  float mean = (cnt > 0) ? sum / (float)cnt : 0.0f;

  float max_psi = -1e30f, I_at_max = 0.0f;
  for (int i = exp->edge_idx[0]; i < exp->edge_idx[1]; i++)
  {
    float psi_c = psi_buf[i] - mean;
    if (psi_c > max_psi)
    {
      max_psi = psi_c;
      I_at_max = I_buf[i];
    }
  }

  if (max_psi < -1e29f) return false;

  exp->sum_max_psi += max_psi;
  exp->sum_max_I += I_at_max;
  exp->repeat_count++;

  if (exp->repeat_count >= exp->repeat_times)
  {
    *avg_psi = exp->sum_max_psi / (float)exp->repeat_times;
    *avg_I = exp->sum_max_I / (float)exp->repeat_times;
    exp->sum_max_psi = exp->sum_max_I = 0.0f;
    exp->repeat_count = 0;
    return true;  // 本轮结束
  }
  return false;  // 还要继续
}

// 切换到下一 Imax
static bool next_imax(FluxExperiment_t* exp)
{
  int curI = (int)exp->inj.Imax;
  int newI = (exp->step_index == 0) ? exp->start_I : curI + exp->step_dir;

  bool finished = false;
  if (exp->step_dir < 0)
  {
    finished = (newI < exp->final_I);
  }
  else
  {
    finished = (newI > exp->final_I);
  }

  if (finished || exp->step_index >= exp->max_steps) return false;

  exp->inj.Imax = (float)newI;
  exp->pos = exp->edge_count = 0;
  return true;
}
