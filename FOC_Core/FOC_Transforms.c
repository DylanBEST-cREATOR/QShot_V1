#include "FOC_Transforms.h"


//static __inline void AL_Clark(volatile Clark_t *CLARK) 
void AL_Clark(volatile Clark_t *CLARK){
    // alpha 轴等于 a 相
    CLARK->I_alpha = CLARK->I_a;
    // 1/sqrt(3) ≈ 0.57735026919f
    CLARK->I_beta = (CLARK->I_a + 2.0f * CLARK->I_b) * 0.57735026919f;

}


// 修改 Park 和 InvPark，改为直接传入 sin/cos 值，避免内部重复计算
//static __inline void AL_Park(volatile Park_t *PARK, float sin_t, float cos_t) 
void AL_Park(volatile Park_t *PARK, float sin_t, float cos_t){
    PARK->I_d =  PARK->I_alpha * cos_t + PARK->I_beta * sin_t;
    PARK->I_q = -PARK->I_alpha * sin_t + PARK->I_beta * cos_t;
}



//static __inline void AL_InvPark(volatile Ip_t *IPARK, float sin_t, float cos_t) 
void AL_InvPark(volatile Ip_t *IPARK, float sin_t, float cos_t) {
    IPARK->U_alpha = IPARK->d * cos_t - IPARK->q * sin_t;
    IPARK->U_beta  = IPARK->d * sin_t + IPARK->q * cos_t;
}
