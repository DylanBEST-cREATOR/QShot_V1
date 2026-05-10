#ifndef __FOC_TRANSFORMS_H
#define __FOC_TRANSFORMS_H
#include "FOC_Qformat.h"
#include "FOC_Svpwm.h"
#include "FOC_Math.h"


typedef struct {

    q16_t alpha;  // Alpha 轴电流 (Q16 标幺值)

    q16_t beta;   // Beta 轴电流 (Q16 标幺值)

} Clarke_t;



typedef struct {

    q16_t d;      // d 轴分量 (Q16 标幺值)

    q16_t q;      // q 轴分量 (Q16 标幺值)

} Park_t;


// 1/sqrt(3) 的 Q16 定点常数
#define INV_SQRT3_Q16   (37837) 

/**
 * @brief  等幅值克拉克（Clarke）变换 (Q16 高性能生产版)
 * @param  current: 输入三相电流指针 (只读 const)
 * @param  clarke:  输出两相静止坐标指针
 */
static __inline void FT_Clarke_Transform(const Current_Sense_t *current, Clarke_t *clarke) {
    // 优化点：将输入一次性读入 CPU 寄存器，避免后续重复寻址带来的总线开销
    int32_t ia = current->Ia;
    int32_t ib = current->Ib;
    int32_t ic = current->Ic;

    clarke->alpha = (q16_t)ia;

    // 优化点：在 32 位寄存器内先完成高效率减法，再强转 64 位进行大数乘法
    int32_t diff = ib - ic;
    int64_t temp_beta = (int64_t)diff * INV_SQRT3_Q16;
    clarke->beta = (q16_t)(temp_beta >> 16);
}

/**
 * @brief  帕克（Park）变换 (Q16 高性能生产版)
 * @param  clarke: 输入两相静止坐标指针 (只读 const)
 * @param  sincos: 输入三角函数指针 (只读 const)
 * @param  park:   输出旋转坐标指针
 */
static __inline void FT_Park_Transform(const Clarke_t *clarke, const SinCos_t *sincos, Park_t *park) {
    // 优化点：将零散的指针成员全部“定格”到 CPU 通用寄存器 (R0-R12) 中
    int32_t alpha = clarke->alpha;
    int32_t beta  = clarke->beta;
    int32_t sin_t = sincos->s;
    int32_t cos_t = sincos->c;

    // Compute d-axis: Id = Ialpha * cos + Ibeta * sin
    // 优化点：寄存器变量相乘，编译器极易生成 ARM Cortex-M 的单周期 SMLAL (64位乘加) 指令
    int64_t temp_d = ((int64_t)alpha * cos_t) + ((int64_t)beta * sin_t);
    park->d = (q16_t)(temp_d >> 16);

    // Compute q-axis: Iq = Ibeta * cos - Ialpha * sin
    // 优化点：调整了计算顺序，契合乘减指令的指令流
    int64_t temp_q = ((int64_t)beta * cos_t) - ((int64_t)alpha * sin_t);
    park->q = (q16_t)(temp_q >> 16);
}

/**
 * @brief  反帕克（Inverse Park）变换 (Q16 高性能生产版)
 * @param  park:   输入旋转坐标电压指针 (只读 const)
 * @param  sincos: 输入三角函数指针 (只读 const)
 * @param  clarke: 输出两相静止坐标电压指针
 */
static __inline void FT_InvPark_Transform(const Park_t *park, const SinCos_t *sincos, Clarke_t *clarke) {
    // 优化点：寄存器缓存
    int32_t vd    = park->d;
    int32_t vq    = park->q;
    int32_t sin_t = sincos->s;
    int32_t cos_t = sincos->c;

    // Compute Valpha: Valpha = Vd * cos - Vq * sin
    int64_t temp_alpha = ((int64_t)vd * cos_t) - ((int64_t)vq * sin_t);
    clarke->alpha = (q16_t)(temp_alpha >> 16);

    // Compute Vbeta: Vbeta = Vd * sin + Vq * cos
    int64_t temp_beta = ((int64_t)vd * sin_t) + ((int64_t)vq * cos_t);
    clarke->beta = (q16_t)(temp_beta >> 16);
}



#endif
