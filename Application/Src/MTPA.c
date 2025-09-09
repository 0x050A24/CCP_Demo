#include "MTPA.h"
#include <math.h>

// ==================== 全局变量 ====================
MTPA_Point MTPA_table[MAX_POINTS];
int point_count = 0;
float Id_mtpa = 0;
static float last_Iq_meas = 0.0; // 上一次测量的 Iq
static int stable_counter = 0;    // 稳定计数器

// ==================== MTPA 求解函数(用户需实现) ====================

float a_d = 5.59756, b_d = 5.15426, m = 5.0;
float a_q = 6.306, b_q = 171.571, n = 1.0;
float c_coeff = 35.90, h = 1.0, j = 0.0;

// ---------- 电流模型 (ψd, ψq → id, iq) ----------
float calc_id(float psi_d, float psi_q) {
    float term = a_d + b_d * pow(fabs(psi_d), m)
                + (c_coeff / (j + 2.0)) * pow(fabs(psi_d), h) * pow(fabs(psi_q), j + 2.0);
    return term * psi_d;
}

float calc_iq(float psi_d, float psi_q) {
    float term = a_q + b_q * pow(fabs(psi_q), n)
                + (c_coeff / (h + 2.0)) * pow(fabs(psi_q), j) * pow(fabs(psi_d), h + 2.0);
    return term * psi_q;
}

// ---------- 转矩计算 ----------
float calc_torque(float psi_d, float psi_q, float id, float iq) {
    return 1.5 * POLE_PAIRS * (psi_d * iq - psi_q * id);
}

// ---------- 目标函数 J = T / Is ----------
float objective(float psi, float theta) {
    float psi_d = psi * cos(theta);
    float psi_q = psi * sin(theta);

    float id = calc_id(psi_d, psi_q);
    float iq = calc_iq(psi_d, psi_q);

    float Is = sqrt(id * id + iq * iq);
    if (Is < 1e-6) return 0.0; // 避免除零

    float T = calc_torque(psi_d, psi_q, id, iq);
    return T / Is;
}

// ---------- 黄金分割搜索 MTPA ----------
float MTPA_find_theta(float psi) {
    float left = 0.0;
    float right = M_PI / 2.0;

    float c = right - (right - left) * GOLDEN_RATIO;
    float d = left + (right - left) * GOLDEN_RATIO;

    float Jc = objective(psi, c);
    float Jd = objective(psi, d);

    while ((right - left) > DELTA_THETA) {
        if (Jc > Jd) {
            right = d;
            d = c;
            Jd = Jc;
            c = right - (right - left) * GOLDEN_RATIO;
            Jc = objective(psi, c);
        } else {
            left = c;
            c = d;
            Jc = Jd;
            d = left + (right - left) * GOLDEN_RATIO;
            Jd = objective(psi, d);
        }
    }

    return 0.5 * (left + right); // 最优角
}


// 输入：磁链幅值 Psi_s
// 输出：对应 MTPA 点 (Iq,Iq)
MTPA_Point calc_MTPA_point(float PSI_s)
 {
    MTPA_Point p;
    p.Psi_s = PSI_s;
    float theta_opt = MTPA_find_theta(PSI_s);
    float psi_d = p.Psi_s * cos(theta_opt);
    float psi_q = p.Psi_s * sin(theta_opt);
    p.PSI_theta = theta_opt;

    float id = calc_id(psi_d, psi_q);
    float iq = calc_iq(psi_d, psi_q);
    
    p.Iq = iq;
    p.Id = id;

    p.is_fixed = false;
    return p;
}

// ==================== 插点并排序 ====================
static void insert_point(MTPA_Point newP) {
    if (point_count < MAX_POINTS) {
        MTPA_table[point_count++] = newP;
    } else {
        // 找到最远的非固定点，替换掉
        int far_idx = -1;
        float max_dist = -1;
        for (int j = 0; j < point_count; j++) {
            if (!MTPA_table[j].is_fixed) {
                float dist = fabs(MTPA_table[j].Iq - newP.Iq);
                if (dist > max_dist) {
                    max_dist = dist;
                    far_idx = j;
                }
            }
        }
        if (far_idx >= 0) {
            MTPA_table[far_idx] = newP;
        }
    }

    // 排序（按 Iq 从小到大）
    for (int i = 0; i < point_count - 1; i++) {
        for (int j = 0; j < point_count - 1 - i; j++) {
            if (MTPA_table[j].Iq > MTPA_table[j + 1].Iq) {
                MTPA_Point tmp = MTPA_table[j];
                MTPA_table[j] = MTPA_table[j + 1];
                MTPA_table[j + 1] = tmp;
            }
        }
    }
}

// ==================== MTPA 更新函数 (带稳定性检测) ====================
void MTPA_update(float Iq_meas) 
{
    // 1. 电流稳定性检测
    if (fabs(Iq_meas - last_Iq_meas) < DELTA_STABLE) {
        stable_counter++;
    } 
    else {
        stable_counter = 0;
    }
    last_Iq_meas = Iq_meas;

   
    // 2. 遍历查找 Iq 所在的区间
    for (int i = 0; i < point_count - 1; i++) {
        if (Iq_meas >= MTPA_table[i].Iq && Iq_meas <= MTPA_table[i + 1].Iq) {
            float psi_left  = MTPA_table[i].Psi_s;
            float psi_right = MTPA_table[i + 1].Psi_s;
            float alpha = 0.5; // 低通滤波系数
            float mtpa_ratio = (Iq_meas - MTPA_table[i].Iq) / (MTPA_table[i+1].Iq - MTPA_table[i].Iq);
            float Id_mtpa_new  = MTPA_table[i].Id + mtpa_ratio * (MTPA_table[i+1].Id - MTPA_table[i].Id);// 线性插值
            static float Id_mtpa_old = 0.0f;
            Id_mtpa = alpha * Id_mtpa_new + (1 - alpha) * Id_mtpa_old;
            Id_mtpa_old = Id_mtpa;

            // 区间过小，认为收敛
            if (fabs(MTPA_table[i + 1].Iq -MTPA_table[i].Iq ) < DELTA_I) {
                return;
            }
            if (fabs(MTPA_table[i].Iq - Iq_meas) < delta_iq ||
                fabs(MTPA_table[i + 1].Iq - Iq_meas) < delta_iq)
            {
                return;
            } 
            if (stable_counter < STABLE_COUNT) 
            {
            return; // 电流未稳定，不更新
            }
            stable_counter = 0; // 重置计数器

            // 二分：取中点 Psi
            float psi_mid = 0.5 * (psi_left + psi_right);
            MTPA_Point newP = calc_MTPA_point(psi_mid);

            // 如果新点 Iq 已经和测量 Iq 接近，认为收敛
           

            // 插入新点
            insert_point(newP);
            return;
        }
    }
}

// ==================== 初始化三点表 ====================
void MTPA_init(float psi_min,float psi_1,float psi_2,float psi_3, float psi_mid,float psi_4,float psi_5, float psi_max) 
{
    MTPA_table[0] = calc_MTPA_point(psi_min); MTPA_table[0].is_fixed = true;
    MTPA_table[1] = calc_MTPA_point(psi_1); MTPA_table[1].is_fixed = true;
    MTPA_table[2] = calc_MTPA_point(psi_2); MTPA_table[2].is_fixed = true;
    MTPA_table[3] = calc_MTPA_point(psi_3); MTPA_table[3].is_fixed = true;
    MTPA_table[4] = calc_MTPA_point(psi_mid); MTPA_table[4].is_fixed = true;
    MTPA_table[5] = calc_MTPA_point(psi_4); MTPA_table[5].is_fixed = true;
    MTPA_table[6] = calc_MTPA_point(psi_5); MTPA_table[6].is_fixed = true;
    MTPA_table[7] = calc_MTPA_point(psi_max); MTPA_table[7].is_fixed = true;
    point_count = 8;
}