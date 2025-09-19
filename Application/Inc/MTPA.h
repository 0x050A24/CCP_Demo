#ifndef MTPA_H
#define MTPA_H

#include <stdint.h>
#include <stdatomic.h>
#include <identification.h>
#ifndef __THREAD_FENCE
  #define __THREAD_FENCE()  atomic_thread_fence(memory_order_seq_cst)
#endif
#include "arm_math.h"/* CMSIS-DSP math */ // IWYU pragma: export
#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)
// ==================== 参数定义 ====================
#define MAX_POINTS    14       // 表最多存 10 个点 (8 固定 + 2 动态)
#define DELTA_I    0.5F  // Iq 区间收敛阈值
#define delta_iq      0.1F    // Iq 收敛阈值 (A)
#define DELTA_STABLE  0.1F    // 电流稳定阈值 (A)
#define STABLE_COUNT  10      // 连续稳定采样次数，才更新

// ==================== 数据结构 ====================
typedef struct {
    float Id;     // d 轴电流
    float Iq;     // q 轴电流
    float Psi_s;  // 磁链幅值
    float PSI_theta; // 最优角
    uint8_t is_fixed; // 是否为固定点（初始三点）
} MTPA_Point;

// ==================== 全局变量 ====================
extern MTPA_Point MTPA_table[MAX_POINTS];
extern int point_count;
extern float Id_mtpa;
extern volatile int mtpa_req_pending;

// ==================== 对外接口 ====================
void MTPA_init(float psi_min,float psi_1,float psi_2,float psi_3, float psi_mid,float psi_4,float psi_5, float psi_6,float psi_7,float psi_8,float psi_9, float psi_max);
//void MTPA_update(float Iq_meas);
/*** —— 中断↔主循环的接口 —— ***/
void MTPA_update_ISR(float Iq_meas);

// 主循环侧：轮询服务（收到请求才计算），建议 100–200 Hz 调一次
void MTPA_service_tick(void);


// 用户需要在 MTPA.c 里实现
MTPA_Point calc_MTPA_point(float Psi_s);



// ---------- 参数定义 ----------
#define DELTA_THETA 0.02F     // 黄金分割法收敛精度
#define GOLDEN_RATIO 0.618F
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
void post_req_from_isr(float psi_mid, float iq_meas);
void assign_parameters_from_LLS(LLS_Result_t res);

#endif // MTPA_H
