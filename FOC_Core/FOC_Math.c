#include "FOC_Math.h"

//static __inline void  AL_fastSinCos(float angle, SinCos_t *sc) 
void  AL_fastSinCos(float angle, SinCos_t *sc){


    float x = angle;

    if (x > AL_PI) x -= AL_2PI;



    // --- 正弦计算 ---

    float abs_x = (x < 0) ? -x : x;

    float y = 1.27323954f * x - 0.405284735f * x * abs_x;

    float abs_y = (y < 0) ? -y : y;

    sc->s = 0.225f * (y * abs_y - y) + y;



    // --- 余弦计算 (利用 cos(x) = sin(x + pi/2)) ---

    x += 1.57079632679f;

    if (x > AL_PI) x -= AL_2PI; // 再次边界检查

    abs_x = (x < 0) ? -x : x;

    y = 1.27323954f * x - 0.405284735f * x * abs_x;

    abs_y = (y < 0) ? -y : y;

    sc->c = 0.225f * (y * abs_y - y) + y;

}
