#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#define AL_PI          3.14159265359f

#define AL_2PI         6.28318530718f

#define AL_INV_PI      0.31830988618f

#define AL_INV_PI_SQ   0.10132118364f


typedef struct {

    float s;

    float c;

} SinCos_t;

//static __inline void  AL_fastSinCos(float angle, SinCos_t *sc)
extern  void  AL_fastSinCos(float angle, SinCos_t *sc);


#endif