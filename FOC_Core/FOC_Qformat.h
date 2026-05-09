#ifndef __FOC_QFORMAT_H
#define __FOC_QFORMAT_H 
#include <stdint.h>  // 引入标准整型头文件，确保支持 int32_t 和 int64_t

typedef int32_t  q16_t;

#define Q16_1           (65536)                     // 1.0 的 Q16 表示
#define _Q16(F)         ((q16_t)((F) * 65536.0f))   // 浮点转 Q16
#define _Q16toF(Q)      ((float)(Q) / 65536.0f)     // Q16 转浮点

// 核心乘法：防溢出，必须转 64 位
#define _Q16mpy(A, B)   ((q16_t)(((int64_t)(A) * (B)) >> 16))

// 饱和限幅
static __inline q16_t _Q16sat(int64_t val, q16_t max, q16_t min) {
    if (val > max) return max;
    if (val < min) return min;
    return (q16_t)val;
}



#define BASE_CURRENT      (30.0f)     // 电流基准：30 A
#define BASE_VOLTAGE      (24.0f)     // 电压基准：24 V
#define BASE_FREQ         (583.0f)    // 频率基准：583 Hz

// ==================== 2. 物理值转 Q16 标幺值宏 ====================
// 输入：浮点物理值 (如 1.5A, 12V)
// 输出：Q16 格式的标幺值
#define PHYS_TO_I_PU(i)   ((q16_t)(((float)(i) / BASE_CURRENT) * 65536.0f))
#define PHYS_TO_V_PU(v)   ((q16_t)(((float)(v) / BASE_VOLTAGE) * 65536.0f))
#define PHYS_TO_F_PU(f)   ((q16_t)(((float)(f) / BASE_FREQ) * 65536.0f))

// ==================== 3. Q16 标幺值还原为浮点物理值宏 ====================
// 输入：Q16 格式的标幺值
// 输出：浮点物理值
#define I_PU_TO_PHYS(i_pu)  (((float)(i_pu) / 65536.0f) * BASE_CURRENT)
#define V_PU_TO_PHYS(v_pu)  (((float)(v_pu) / 65536.0f) * BASE_VOLTAGE)
#define F_PU_TO_PHYS(f_pu)  (((float)(f_pu) / 65536.0f) * BASE_FREQ)



#endif // Q16_MATH_H