#ifndef MTPA_H
#define MTPA_H

#include <stdint.h>
#include <stdatomic.h>
#include <stdbool.h>
#ifndef __THREAD_FENCE
  #define __THREAD_FENCE()  atomic_thread_fence(memory_order_seq_cst)
#endif
#include "arm_math.h"/* CMSIS-DSP math */ // IWYU pragma: export
#define COS(x) arm_cos_f32(x)
#define SIN(x) arm_sin_f32(x)


#ifdef __cplusplus
extern "C" {
#endif

/* ------------------ 可调宏/常量 ------------------ */
#define MTPA_TABLE_POINTS    51    /* 0..50 共 51 点（若想 50 点可改为 50） */
#define MTPA_PSI_MAX         1.10f /* 内环磁链幅值上限（题目要求 0-1.1） */
#define MTPA_PSI_SCAN_STEPS  50   /* 内层扫描初筛步数（越大越精细，耗时越多） */
#define MTPA_PSI_BISECT_TOL  1e-5f /* 二分求解 Psi 的容忍（磁链或转矩） */
#define MTPA_TH_TOL          1e-4f /* 最终 gamma 搜索精度（弧度） */
#define MTPA_TH_MAX_ITER     80    /* 黄金分割最大迭代次数 */
#define MTPA_PSI_MIN         0.0f
#define MTPA_MIN_IS          1e-9f /* 防止除零 */

/* 极对数（用于转矩公式）与因子 */
#define POLE_PAIRS 2
#define KAPPA (1.5f * (float)POLE_PAIRS)   /* 3/2 * p */

/* ------------------ MTPA 点结构 ------------------ */
typedef struct {
    float T_req;   /* 给定转矩 T* */
    float Psi_s;   /* 磁链幅值 */
    float gamma;   /* 最优角度（rad） */
    float Id;      /* d轴电流 */
    float Iq;      /* q轴电流 */
    bool  valid;   /* 是否找到可行解 */
} MTPA_Point;

extern MTPA_Point mtpa_table[MTPA_TABLE_POINTS];
/* ------------------ 对外接口 ------------------ */

void MTPA_Get_Parameter(float ad0, float add, float aq0, float aqq, float adq);
/* 初始化/构建表：T_min..T_max 会均匀分配到 MTPA_TABLE_POINTS 点（含端点）
   如果要 1..50 且共 50 个点，可调整函数调用参数 */
void MTPA_build_table(MTPA_Point table[], int n_points, float T_min, float T_max);
/* 将 Id,Iq 存入表后可用该函数在运行时插值（按 Iq 查找并线性内插）：
   给定目标 Iq_ref，得到 Id_ref（用于实时控制环） */
void MTPA_interp_by_Iq(const MTPA_Point table[], int n_points, float Iq_ref, float *Id_ref, float *Iq_out);

#ifdef __cplusplus
}
#endif

#endif // MTPA_H
