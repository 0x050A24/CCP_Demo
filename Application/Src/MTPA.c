#include "MTPA.h"
#include <string.h>
#include <math.h>

typedef struct { float psi_mid; float iq_meas; } MTPA_Req;
static MTPA_Req    mtpa_req_mb;
  
// ==================== 全局变量 ====================
MTPA_Point MTPA_table[MAX_POINTS];
int point_count = 0;
float Id_mtpa = 0.0f;
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

    float Is =sqrt( id * id + iq * iq);
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

    p.is_fixed = 0;
    return p;
}

// ==================== 插点并排序 ====================
//static void insert_point(MTPA_Point newP) {
//    if (point_count < MAX_POINTS) {
//        MTPA_table[point_count++] = newP;
//    } else {
        // 找到最远的非固定点，替换掉
//        int far_idx = -1;
  //      float max_dist = -1;
  //      for (int j = 0; j < point_count; j++) {
  //          if (!MTPA_table[j].is_fixed) {
  //              float dist = fabs(MTPA_table[j].Iq - newP.Iq);
  //              if (dist > max_dist) {
   //                 max_dist = dist;
  //                  far_idx = j;
   //             }
   //         }
    //    }
   //     if (far_idx >= 0) {
    //        MTPA_table[far_idx] = newP;
    //    }
    //}

    // 排序（按 Iq 从小到大）
    //for (int i = 0; i < point_count - 1; i++) {
    //    for (int j = 0; j < point_count - 1 - i; j++) {
    //        if (MTPA_table[j].Iq > MTPA_table[j + 1].Iq) {
    //            MTPA_Point tmp = MTPA_table[j];
    //            MTPA_table[j] = MTPA_table[j + 1];
    //            MTPA_table[j + 1] = tmp;
    //        }
    //    }
  //  }
//}

/*** 两份表 ***/
static MTPA_Point tblA[MAX_POINTS], tblB[MAX_POINTS];
static MTPA_Point *active_tbl = tblA;  static int active_cnt = 0;   // ISR 只读
static MTPA_Point *build_tbl  = tblB;  static int build_cnt  = 0;   // 主循环只写

/*** 版本号：偶数=稳定；发布时先++(奇)，写指针，再++(偶) ***/
static volatile uint32_t mtpa_seq = 0;

/*** 给ISR的“一致快照” ***/
static inline void mtpa_snapshot_for_isr(MTPA_Point **t, int *n){
    uint32_t s1, s2;
    do {
        s1 = mtpa_seq; __DMB();
        *t = active_tbl; *n = active_cnt;
        __DMB(); s2 = mtpa_seq;
    } while ((s1 != s2) || (s1 & 1U));   // 若发布中(奇数)或不一致，则重读
}

/*** 主循环发布（不关中断） ***/
static inline void mtpa_publish_from_main(void){
    mtpa_seq++; __DMB();                 // 进入发布（奇数）
    active_tbl = build_tbl; active_cnt = build_cnt;
    __DMB(); mtpa_seq++; 
    if(mtpa_seq>=60){mtpa_seq=0;}
                   // 发布完成（偶数）
    // 交换角色，清空 build 计数
    build_tbl  = (build_tbl == tblB) ? tblA : tblB;
    //build_cnt  = 0;
}

/*** 在 build 上插入并按 Iq 排序 ***/

static void insert_point_on_build(MTPA_Point newP)
{
    if (build_cnt < MAX_POINTS){
        build_tbl[build_cnt++] = newP;
    }else{
        int far_idx=-1; float maxd=-1.f;
        for (int j=0;j<build_cnt;j++){
            if (!build_tbl[j].is_fixed){
                float d = fabsf(build_tbl[j].Iq - newP.Iq);
                if (d>maxd){ maxd=d; far_idx=j; }
            }
        }
        if (far_idx>=0) build_tbl[far_idx]=newP;
    }
    // 冒泡排序（仍然在主循环执行）
    for (int i=0;i<build_cnt-1;i++){
        for (int j=0;j<build_cnt-1-i;j++){
            if (build_tbl[j].Iq > build_tbl[j+1].Iq){
                MTPA_Point t = build_tbl[j]; build_tbl[j]=build_tbl[j+1]; build_tbl[j+1]=t;
            }
        }
    }
}



// ==================== MTPA 更新函数 (带稳定性检测) ====================
/*void MTPA_update(float Iq_meas) 
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
            if (stable_counter < STABLE_COUNT) 
            {
            return; // 电流未稳定，不更新
            }
            // 区间过小，认为收敛
            if (fabs(MTPA_table[i + 1].Iq -MTPA_table[i].Iq ) < DELTA_I) {
                return;
            }
            if (fabs(MTPA_table[i].Iq - Iq_meas) < delta_iq ||
                fabs(MTPA_table[i + 1].Iq - Iq_meas) < delta_iq)
            {
                return;
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
}*/

// ==================== 初始化三点表 ====================

void MTPA_init(float psi_min,float psi_1,float psi_2,float psi_3, float psi_mid,float psi_4,float psi_5, float psi_6,float psi_7,float psi_8,float psi_9, float psi_max) 
{
   /* MTPA_table[0] = calc_MTPA_point(psi_min); MTPA_table[0].is_fixed = true;
    MTPA_table[1] = calc_MTPA_point(psi_1); MTPA_table[1].is_fixed = true;
    MTPA_table[2] = calc_MTPA_point(psi_2); MTPA_table[2].is_fixed = true;
    MTPA_table[3] = calc_MTPA_point(psi_3); MTPA_table[3].is_fixed = true;
    MTPA_table[4] = calc_MTPA_point(psi_mid); MTPA_table[4].is_fixed = true;
    MTPA_table[5] = calc_MTPA_point(psi_4); MTPA_table[5].is_fixed = true;
    MTPA_table[6] = calc_MTPA_point(psi_5); MTPA_table[6].is_fixed = true;
    MTPA_table[7] = calc_MTPA_point(psi_max); MTPA_table[7].is_fixed = true;
    point_count = 8;*/
     float psis[12]={psi_min,psi_1,psi_2,psi_3,psi_mid,psi_4,psi_5,psi_6,psi_7,psi_8,psi_9,psi_max};
    build_cnt = 0;
    for(int k=0;k<12;k++){ MTPA_Point p = calc_MTPA_point(psis[k]); p.is_fixed=1; insert_point_on_build(p); }
    mtpa_publish_from_main();   // 活动表就绪
}
/*** —— 中断读取时的简易插值（只读 active 表） —— ***/
 void interp_IdIq_by_Iq(const MTPA_Point *T, int N, float iq_meas, float *Id_ref, float *Iq_ref){
    if (N<2){ *Id_ref=0; *Iq_ref=iq_meas; return; }
    int i;
    for (i=0;i<N-1;i++) if (iq_meas>=T[i].Iq && iq_meas<=T[i+1].Iq) break;
    if (i>=N-1) i=N-2;
    float w = (iq_meas - T[i].Iq) / (T[i+1].Iq - T[i].Iq + 1e-9f);
    *Id_ref = T[i].Id + w*(T[i+1].Id - T[i].Id);
    *Iq_ref = iq_meas; // 或者也插值Iq
}
    
extern MTPA_Point *active_tbl; extern int active_cnt;
extern MTPA_Point *build_tbl;  extern int build_cnt;
extern void mtpa_publish_from_main(void);
extern void insert_point_on_build(MTPA_Point p);



void MTPA_service_tick(void)
{
    if (!mtpa_req_pending) return;

    // 取出请求并清零（不关中断，先读内容再清标志）
    float psi_mid = mtpa_req_mb.psi_mid;
    __DMB(); mtpa_req_pending = 0;

    // 第一次进入时，把 active 拷到 build（只需做一次）
    static int cloned = 0;
    if (!cloned){ for(int i=0;i<active_cnt;i++) build_tbl[i]=active_tbl[i];
                  build_cnt = active_cnt; cloned = 1; }

    // —— 主循环里“重活”：黄金分割 & 新点插入（冒泡排序保留） ——
    MTPA_Point newP = calc_MTPA_point(psi_mid);
    insert_point_on_build(newP);

    // —— 发布：把 build 切成 active（不关中断） ——
    mtpa_publish_from_main();
}
void MTPA_update_ISR(float Iq_meas)
{
    // 取活动表的快照（不关中断）
    MTPA_Point *T; int N; mtpa_snapshot_for_isr(&T, &N);
    if (N < 2) return;

    // 稳定性检测（照旧）
    if (fabsf(Iq_meas - last_Iq_meas) < DELTA_STABLE) stable_counter++; else stable_counter=0;
    last_Iq_meas = Iq_meas;

    // 遍历查找 Iq 区间（照旧）
    for (int i = 0; i < N - 1; i++) {
        if (Iq_meas >= T[i].Iq && Iq_meas <= T[i + 1].Iq) {

            // 线性插值 Id_mtpa（照旧）
            float w = (Iq_meas - T[i].Iq) / (T[i+1].Iq - T[i].Iq + 1e-9f);
            float Id_mtpa_new = T[i].Id + w * (T[i+1].Id - T[i].Id);
            static float Id_mtpa_old = 0.f;
            float alpha = 0.25f;
            Id_mtpa = alpha*Id_mtpa_new + (1-alpha)*Id_mtpa_old;
            Id_mtpa_old = Id_mtpa;

            // 早退条件（照旧）
            if (stable_counter < STABLE_COUNT) return;
            if (fabsf(T[i+1].Iq - T[i].Iq) < DELTA_I) return;
            if (fabsf(T[i].Iq - Iq_meas) < delta_iq ||fabsf(T[i+1].Iq - Iq_meas) < delta_iq) return;

            // —— 只“发指令”，不做计算 —— 
            float psi_mid = 0.5f * (T[i].Psi_s + T[i+1].Psi_s);
            if (!mtpa_req_pending) { post_req_from_isr(psi_mid, Iq_meas); }
            stable_counter = 0;
            return;
        }
    }
}
// 简单邮箱（单生产者=ISR，单消费者=主循环）

volatile int mtpa_req_pending = 0;


void post_req_from_isr(float psi_mid, float iq_meas){
    mtpa_req_mb.psi_mid = psi_mid;
    mtpa_req_mb.iq_meas = iq_meas;
    __DMB(); mtpa_req_pending = 1;   // 保证先写内容再置位
}

extern void mtpa_snapshot_for_isr(MTPA_Point **t, int *n);   // 来自 MTPA_table.c
extern void interp_IdIq_by_Iq(const MTPA_Point *T, int N, float iq_meas, float *Id_ref, float *Iq_ref);
