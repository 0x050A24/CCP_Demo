#include "MTPA.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
/*----------- 辅助宏 -----------*/
#ifndef MIN
  #define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
  #define MAX(a,b) (((a)>(b))?(a):(b))
#endif

/* 给定单个 T，计算 MTPA 点（主函数：调用可嵌入初始化）
   返回 true 表示找到有效点并写入 out_p；否则返回 false（不可行） */
static bool MTPA_compute_for_T(float T_req, MTPA_Point *out_p);

/* 给定 Psi_s、gamma 计算 id, iq, Te（模型来自你给出的拟合函数） */
static void MTPA_model_idiq(float psi_d, float psi_q, float *id, float *iq);
static float MTPA_calc_torque(float psi_d, float psi_q, float id, float iq);

/* ------------- 你的拟合模型参数（可以修改/从外部注入） ------------- */
/* 与用户给定参数一致 */
// extern  float a_d = 6.019F, b_d = 4.3238F, m = 5.0F;
// float a_q = 10.524F, b_q = 128.6657F, n = 1.0F;
// float c_coeff = 62.6F, h = 1.0F, j = 0.0F;
extern MTPA_Point mtpa_table[MTPA_TABLE_POINTS]= {0};
static float a_d = 5.59756, b_d = 5.15426, m = 5.0;
static float a_q = 6.306, b_q = 171.571, n = 1.0;
static float c_coeff = 35.90, h = 1.0, j = 0.0;

void MTPA_Get_Parameter(float ad0, float add, float aq0, float aqq, float adq)
{
    a_d = ad0;
    b_d = add;
    a_q = aq0;
    b_q = aqq;
    c_coeff = adq;
}

/* -------------- 模型实现： psi_d, psi_q -> id, iq -------------- */
/* 使用题主给定的模型（包含绝对值次幂项） */
void MTPA_model_idiq(float psi_d, float psi_q, float *id, float *iq)
{
    /* term_d = a_d + b_d * |psi_d|^m + (c/(j+2)) * |psi_d|^h * |psi_q|^(j+2) */
    float abs_pd = fabsf(psi_d);
    float abs_pq = fabsf(psi_q);

    float term_d = a_d + b_d * powf(abs_pd, m);
    term_d += (c_coeff / (j + 2.0f)) * powf(abs_pd, h) * powf(abs_pq, j + 2.0f);

    float term_q = a_q + b_q * powf(abs_pq, n);
    term_q += (c_coeff / (h + 2.0f)) * powf(abs_pq, j) * powf(abs_pd, h + 2.0f);

    *id = term_d * psi_d;
    *iq = term_q * psi_q;
}

/* -------------- 转矩计算 -------------- */
float MTPA_calc_torque(float psi_d, float psi_q, float id, float iq)
{
    /* Te = (3/2) * p * (psi_d * iq - psi_q * id) */
    return KAPPA * (psi_d * iq - psi_q * id);
}

/* -------------- 内层：给定 (psi, gamma) 计算 Te 和 Id,Iq,Is -------------- */
static void compute_at_psi_gamma(float psi, float gamma, float *Te, float *Id, float *Iq, float *Is)
{
    float psi_d = psi * cosf(gamma);
    float psi_q = psi * sinf(gamma);
    float id_local, iq_local;
    MTPA_model_idiq(psi_d, psi_q, &id_local, &iq_local);
    float Te_local = MTPA_calc_torque(psi_d, psi_q, id_local, iq_local);
    float Is_local = sqrtf(id_local * id_local + iq_local * iq_local);
    if (Id) *Id = id_local;
    if (Iq) *Iq = iq_local;
    if (Te) *Te = Te_local;
    if (Is) *Is = Is_local;
}

/* -------------- 内环求 Psi（在固定 gamma 下），返回最小 Psi 使 Te >= T_req -------------- */
/* 方法：
   1) 在 [Psi_min=0, Psi_max] 上均匀采样 MTPA_PSI_SCAN_STEPS 个点，寻找第一个采样区间
      (psi_k, psi_k+1) 使 Te(psi_k) < T <= Te(psi_k+1)；
   2) 若在整个区间 Te(max) < T => 无解（不可行）；
   3) 对找到的区间用二分法求解精确的 psi_root（直到误差/步长满足 MTPA_PSI_BISECT_TOL）。
   返回：true 表示找到根，并把结果放到 out_psi / out_Id / out_Iq / out_Te / out_Is。
*/
static bool find_psi_for_T_at_gamma(float T_req, float gamma,
                                    float *out_psi, float *out_Id, float *out_Iq, float *out_Te, float *out_Is)
{
    const float psi_min = MTPA_PSI_MIN;
    const float psi_max = MTPA_PSI_MAX;
    const int Nscan = MTPA_PSI_SCAN_STEPS;
    float psi_prev = psi_min;
    float Te_prev = 0.0f, Is_prev = 0.0f, Id_prev = 0.0f, Iq_prev = 0.0f;
    /* 计算 psi=0 点 */
    compute_at_psi_gamma(psi_prev, gamma, &Te_prev, &Id_prev, &Iq_prev, &Is_prev);

    /* 若 T_req == 0，最小 psi 就是 0（但电流可能为0）；
       但是按用户要求 T=0 的点会被硬编码处理于上层函数 */
    if (T_req <= 0.0f) {
        if (out_psi) *out_psi = 0.0f;
        if (out_Id) *out_Id = Id_prev;
        if (out_Iq) *out_Iq = Iq_prev;
        if (out_Te) *out_Te = Te_prev;
        if (out_Is) *out_Is = Is_prev;
        return true;
    }

    /* 扫描寻找首个跨越区间 */
    bool found_interval = false;
    float psi_low = 0.0f, psi_high = 0.0f;
    float Te_low = 0.0f, Te_high = 0.0f;
    float Id_low = 0.0f, Iq_low = 0.0f, Is_low = 0.0f;

    for (int k = 1; k <= Nscan; ++k) {
        float t = (float)k / (float)Nscan;
        float psi_k = psi_min + t * (psi_max - psi_min);
        float Te_k, Id_k, Iq_k, Is_k;
        compute_at_psi_gamma(psi_k, gamma, &Te_k, &Id_k, &Iq_k, &Is_k);

        if (Te_prev < T_req && Te_k >= T_req) {
            /* 区间 [psi_prev, psi_k] 包含第一个根 */
            psi_low = psi_prev; psi_high = psi_k;
            Te_low = Te_prev; Te_high = Te_k;
            Id_low = Id_prev; Iq_low = Iq_prev; Is_low = Is_prev;
            found_interval = true;
            break;
        }
        psi_prev = psi_k;
        Te_prev = Te_k; Id_prev = Id_k; Iq_prev = Iq_k; Is_prev = Is_k;
    }

    if (!found_interval) {
        /* 即使在 psi_max 上 Te 也不足，认为该 gamma 不可行（限幅） */
        return false;
    }

    /* 二分法在 [psi_low, psi_high] 中求解 Te(psi) = T_req 精确的 psi_root （取较小根） */
    float left = psi_low, right = psi_high;
    float mid = 0.0f;
    float Te_mid = 0.0f, Id_mid = 0.0f, Iq_mid = 0.0f, Is_mid = 0.0f;
    int guard = 0;
    while ((right - left) > MTPA_PSI_BISECT_TOL && guard < 80) {
        mid = 0.5f * (left + right);
        compute_at_psi_gamma(mid, gamma, &Te_mid, &Id_mid, &Iq_mid, &Is_mid);
        if (Te_mid >= T_req) {
            right = mid;
        } else {
            left = mid;
        }
        guard++;
    }

    /* 输出结果（取最终 right 位置） */
    float psi_root = right;
    compute_at_psi_gamma(psi_root, gamma, out_Te, out_Id, out_Iq, out_Is);
    if (out_psi) *out_psi = psi_root;

    /* 若 Te(root) < T_req（数值问题）则认为不可行 */
    if (out_Te && (*out_Te < T_req - 1e-6f)) return false;

    return true;
}

/* -------------- 外层：给定 T，搜索 gamma 使 Is 最小（黄金分割法） -------------- */
/* 返回 true 并填充 out_p 表示找到可行的最小 Is；否则返回 false（无可行 gamma） */
bool MTPA_compute_for_T(float T_req, MTPA_Point *out_p)
{
    if (!out_p) return false;

    /* 特殊处理：T_req == 0 要求 Iq=0, Id=0.5 按题目要求 */
    if (T_req <= 0.0f) {
        out_p->T_req = 0.0f;
        out_p->Psi_s = 0.0f;      /* 可以置 0 或者最小 */
        out_p->gamma = 0.0f;
        out_p->Id = 0.5f;         /* 题目特定规定 */
        out_p->Iq = 0.0f;
        out_p->valid = true;
        return true;
    }

    /* 黄金分割搜索区间 gamma ∈ [0, pi/2] */
    float a = 0.0f;
    float b = (float)M_PI / 2.0f;
    /* 若需要可以对区间进行收缩（例如 [1° , 89°] 等） */

    /* 初始内点 c, d （按黄金比） */
    const float gr = 0.6180339887498949f;
    float c = b - (b - a) * gr;
    float d = a + (b - a) * gr;

    /* 计算目标值：对于每个 gamma，如果内层可解就得到对应 Psi, Id, Iq 与 Is；
       黄金分割将比较 Is(c) 与 Is(d)，选较小者保留区间。不可行点被记为 Is = +inf。 */
    float Is_c = 1e30f, Is_d = 1e30f;
    float psi_tmp, Id_tmp, Iq_tmp, Te_tmp;
    bool okc = find_psi_for_T_at_gamma(T_req, c, &psi_tmp, &Id_tmp, &Iq_tmp, &Te_tmp, &Is_c);
    if (!okc) Is_c = 1e30f;   /* 不可行 */
    bool okd = find_psi_for_T_at_gamma(T_req, d, &psi_tmp, &Id_tmp, &Iq_tmp, &Te_tmp, &Is_d);
    if (!okd) Is_d = 1e30f;

    int iter = 0;
    float best_Is = 1e30f;
    float best_gamma = 0.0f, best_psi = 0.0f, best_Id = 0.0f, best_Iq = 0.0f, best_Te = 0.0f;

    while ((b - a) > MTPA_TH_TOL && iter < MTPA_TH_MAX_ITER) {
        if (Is_c < Is_d) {
            /* d 可以舍弃，b = d */
            b = d;
            d = c;
            Is_d = Is_c;
            /* 新 c */
            c = b - (b - a) * gr;
            /* eval c */
            bool ok = find_psi_for_T_at_gamma(T_req, c, &psi_tmp, &Id_tmp, &Iq_tmp, &Te_tmp, &psi_tmp/*reuse*/);
            if (ok) {
                /* compute Is properly (we need Id, Iq) */
                compute_at_psi_gamma(psi_tmp, c, &Te_tmp, &Id_tmp, &Iq_tmp, &Is_c);
            } else {
                Is_c = 1e30f;
            }
        } else {
            /* c 可以舍弃，a = c */
            a = c;
            c = d;
            Is_c = Is_d;
            d = a + (b - a) * gr;
            bool ok = find_psi_for_T_at_gamma(T_req, d, &psi_tmp, &Id_tmp, &Iq_tmp, &Te_tmp, &psi_tmp/*reuse*/);
            if (ok) {
                compute_at_psi_gamma(psi_tmp, d, &Te_tmp, &Id_tmp, &Iq_tmp, &Is_d);
            } else {
                Is_d = 1e30f;
            }
        }
        iter++;
    }

    /* 取最终最小点（在 a..b 中采样取最小） */
    int Ncheck = 9;
    float best_local_Is = 1e30f;
    for (int k = 0; k <= Ncheck; ++k) {
        float g = a + (b - a) * ((float)k / (float)Ncheck);
        float psi_out, id_out, iq_out, te_out, is_out;
        bool ok = find_psi_for_T_at_gamma(T_req, g, &psi_out, &id_out, &iq_out, &te_out, &is_out);
        if (ok && is_out < best_local_Is) {
            best_local_Is = is_out;
            best_gamma = g;
            best_psi = psi_out;
            best_Id = id_out;
            best_Iq = iq_out;
            best_Te = te_out;
        }
    }

    if (best_local_Is >= 1e29f) {
        /* 全区间不可行 */
        out_p->valid = false;
        return false;
    }

    /* 填充输出 */
    out_p->T_req = T_req;
    out_p->Psi_s = best_psi;
    out_p->gamma = best_gamma;
    out_p->Id = best_Id;
    out_p->Iq = best_Iq;
    out_p->valid = true;
    return true;
}

/* -------------- 构建整张表（T_min..T_max, 共 n_points） -------------- */
void MTPA_build_table(MTPA_Point table[], int n_points, float T_min, float T_max)
{
    if (n_points <= 0) return;
    /* 均匀分配 T 值（含端点） */
    for (int k = 0; k < n_points; ++k) {
        float t = (float)k / (float)(n_points - 1);
        float T = T_min + t * (T_max - T_min);
        if (T <= 0.0f) {
            /* 按约定：T=0 特殊点 Id=0.5, Iq=0 */
            table[k].T_req = 0.0f;
            table[k].Psi_s = 0.0f;
            table[k].gamma = 0.0f;
            table[k].Id = 0.5f;
            table[k].Iq = 0.0f;
            table[k].valid = true;
        } else {
            MTPA_Point p;
            bool ok = MTPA_compute_for_T(T, &p);
            if (!ok) {
                /* 若做不到，可将点标为 invalid，或尝试回退到限压解（此处简单标 invalid） */
                table[k].T_req = T;
                table[k].valid = false;
                table[k].Psi_s = 0.0f;
                table[k].gamma = 0.0f;
                table[k].Id = 0.0f;
                table[k].Iq = 0.0f;
            } else {
                table[k] = p;
            }
        }
    }
}

/* -------------- 运行期插值：按 Iq 查 Id（中断中可用） -------------- */
void MTPA_interp_by_Iq(const MTPA_Point table[], int n_points, float Iq_ref, float *Id_ref, float *Iq_out)
{
    if (n_points <= 0) {
        if (Id_ref) *Id_ref = 0.0f;
        if (Iq_out) *Iq_out = 0.0f;
        return;
    }
    /* 找到 Iq_ref 所在区间（表按 Iq 不一定排序；此处假定传入表是按 Iq 单调的，
       若你的表不是单调，请事先按 Iq 排序或改为按 Psi/Id 查找。 */
    int i = 0;
    for (i=0;i<n_points-1;i++) if (Iq_ref>=table[i].Iq && Iq_ref<=table[i+1].Iq) break;
    if (i>=n_points-1) i=n_points-2;
    float w = (Iq_ref - table[i].Iq) / (table[i+1].Iq - table[i].Iq + 1e-9f);
    *Id_ref = table[i].Id + w*(table[i+1].Id - table[i].Id);
    *Iq_out = Iq_ref; // 或者也插值Iq
}



