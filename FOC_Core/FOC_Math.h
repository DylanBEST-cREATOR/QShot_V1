#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#define Q16_PI          (65536)     // 1.0 PU 对应 pi (180度)
#define Q16_2PI         (131072)    // 2.0 PU 对应 2*pi (360度)
#define Q16_HALF_PI     (32768)     // 0.5 PU 对应 pi/2 (90度)
#define Q16_0225        (14746)     // 0.225 对应的 Q16 整数 (0.225 * 65536)

#include "FOC_Qformat.h"



typedef struct {
    q16_t Target_Freq_Hz;   // 目标电频率 (Hz, Q16格式)
    uint32_t Ctrl_Freq_Hz;  // 控制环路频率 (PWM中断频率, 如 20000)
    q16_t Angle_Step;       // 每步增加的角度 (Q16格式)
    q16_t Current_Angle;    // 当前输出电角度 (Q16格式, [-65536, 65536])
} VirtualAngle_t;	

typedef struct {
    q16_t s;      // sin(theta) 的 Q16 标幺值
    q16_t c;      // cos(theta) 的 Q16 标幺值
} SinCos_t;

/**
 * @brief  快速正余弦计算 (Q16 定点标幺化版)
 * @param  angle: 输入的电角度标幺值 (Q16 格式，-65536 对应 -pi，65536 对应 pi)
 * @param  sc:    输出的正余弦结构体指针 (输出范围 [-65536, 65536])
 */
static __inline void FM_fastSinCos(volatile q16_t angle, volatile SinCos_t *sc) {
    q16_t x = angle;

    // 边界检查：若大于 pi (65536)，则减去 2*pi (131072) 限制到 [-pi, pi] 之间
    if (x > Q16_PI) x -= Q16_2PI;
	  if (x < -Q16_PI) x += Q16_2PI;

    // ==================== 1. 正弦计算 ====================
    q16_t abs_x = (x < 0) ? -x : x;

    // 核心拟合公式：y = 4*x - 4*x*|x|
    // a. (int64_t)x * abs_x 强转 64 位进行高精度乘法，右移 16 位还原为 Q16 格式下的 (x * |x|)
    // b. << 2 相当于乘以 4
    int64_t x_abs_x = ((int64_t)x * abs_x) >> 16;
    q16_t y = (x << 2) - (q16_t)(x_abs_x << 2);

    q16_t abs_y = (y < 0) ? -y : y;

    // 泰勒修正：sc->s = 0.225 * (y * |y| - y) + y
    // a. y * abs_y 乘积右移 16 位还原 Q16
    // b. (y_abs_y - y) 得到差值
    // c. 乘以外置系数 Q16_0225 (14746)，右移 16 位还原 Q16，最后累加 y
    int64_t y_abs_y = ((int64_t)y * abs_y) >> 16;
    q16_t diff = (q16_t)(y_abs_y - y);
    q16_t term = (q16_t)(((int64_t)diff * Q16_0225) >> 16);

    sc->s = term + y;

    // ==================== 2. 余弦计算 (cos(x) = sin(x + pi/2)) ====================
    x += Q16_HALF_PI; // 角度平移 90 度 (32768)

    if (x > Q16_PI) x -= Q16_2PI; // 再次边界检查

    abs_x = (x < 0) ? -x : x;

    // 重复拟合公式与修正
    x_abs_x = ((int64_t)x * abs_x) >> 16;
    y = (x << 2) - (q16_t)(x_abs_x << 2);

    abs_y = (y < 0) ? -y : y;

    y_abs_y = ((int64_t)y * abs_y) >> 16;
    diff = (q16_t)(y_abs_y - y);
    term = (q16_t)(((int64_t)diff * Q16_0225) >> 16);

    sc->c = term + y;
}

typedef struct {
    int32_t LastOutput; // 上一次的滤波输出 (Q16)
    int32_t Alpha;      // 滤波系数 (Q16, 0~65536 对应 0~1.0)
} LPF_t;

static __inline q16_t FM_LPF_Calculate(LPF_t *lpf, int32_t input) {
    int32_t error = input - lpf->LastOutput;
    // 使用 int64_t 强制转换，防止乘法溢出
    int32_t delta = (int32_t)(((int64_t)lpf->Alpha * error) >> 16);
    lpf->LastOutput += delta;
    return lpf->LastOutput;
}

/**
 * @brief 更新虚拟角度步长
 * @param v_angle: 结构体指针
 * @param target_rpm: 目标机械转速 (Q0)
 * @param pole_pairs: 电机极对数
 */


/**
 * @brief 虚拟角度累加器 (每中断调用一次)
 */
static __inline q16_t FM_VirtualAngle_Update(VirtualAngle_t *v_angle) {
    // 1. 角度累加
    v_angle->Current_Angle += v_angle->Angle_Step;

    // 2. 双边回绕检查 (适配你的 FM_fastSinCos [-PI, PI] 要求)
    if (v_angle->Current_Angle > 65536) {
        v_angle->Current_Angle -= 131072;
    } else if (v_angle->Current_Angle < -65536) {
        v_angle->Current_Angle += 131072;
    }

    return v_angle->Current_Angle;
}

 void FM_VirtualAngle_SetSpeed(VirtualAngle_t *v_angle, int32_t target_rpm, uint8_t pole_pairs);




#endif
