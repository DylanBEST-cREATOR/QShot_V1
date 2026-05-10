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





#endif // Q16_MATH_H
