#ifndef MTPA_H
#define MTPA_H

#include <stdbool.h>

// ==================== 参数定义 ====================
#define MAX_POINTS    10       // 表最多存 10 个点 (8 固定 + 2 动态)
#define DELTA_I    0.5    // Iq 区间收敛阈值
#define delta_iq      0.1    // Iq 收敛阈值 (A)
#define DELTA_STABLE  0.12    // 电流稳定阈值 (A)
#define STABLE_COUNT  8       // 连续稳定采样次数，才更新

// ==================== 数据结构 ====================
typedef struct {
    float Id;     // d 轴电流
    float Iq;     // q 轴电流
    float Psi_s;  // 磁链幅值
    float PSI_theta; // 最优角
    bool is_fixed; // 是否为固定点（初始三点）
} MTPA_Point;

// ==================== 全局变量 ====================
extern MTPA_Point MTPA_table[MAX_POINTS];
extern int point_count;
extern float Id_mtpa;
// ==================== 对外接口 ====================
void MTPA_init(float psi_min,float psi_1,float psi_2,float psi_3, float psi_mid,float psi_4,float psi_5, float psi_max);
void MTPA_update(float Iq_meas);

// 用户需要在 MTPA.c 里实现
MTPA_Point calc_MTPA_point(float Psi_s);


// ---------- 参数定义 ----------
#define DELTA_THETA 0.02     // 黄金分割法收敛精度
#define GOLDEN_RATIO 0.6180339887498949
#define POLE_PAIRS 2         // 极对数 (示例)

// 拟合模型参数（需要根据实验拟合得到）
extern float a_d, b_d, m;
extern float a_q, b_q, n;
extern float c_coeff, h, j;

// ---------- 函数声明 ----------
float calc_id(float psi_d, float psi_q);
float calc_iq(float psi_d, float psi_q);
float MTPA_find_theta(float psi);
float calc_torque(float psi_d, float psi_q, float id, float iq);
float objective(float psi, float theta);
#endif // MTPA_H
