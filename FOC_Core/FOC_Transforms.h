#ifndef __FOC_TRANSFORMS_H
#define __FOC_TRANSFORMS_H

// Clark 变换对象
typedef struct {
    float I_a;  
    float I_b;        
    float I_c;     
    float I_alpha; 
    float I_beta;   
} Clark_t;

// Park 变换对象
typedef struct {
    float I_alpha;  // Input: stationary alpha-axis
    float I_beta;   // Input: stationary beta-axis
    float theta;  // Input: rotor angle (0~2PI)
    float I_d;      // Output: rotating d-axis
    float I_q;      // Output: rotating q-axis
} Park_t;

// 反 Park 变换对象
typedef struct {
    float d;      // Input: rotating d-axis
    float q;      // Input: rotating q-axis
    float theta;  // Input: rotor angle (0~2PI)
    float U_alpha;  // Output: stationary alpha-axis
    float U_beta;   // Output: stationary beta-axis
} Ip_t;

extern void AL_Clark(volatile Clark_t *CLARK);
extern void AL_Park(volatile Park_t *PARK, float sin_t, float cos_t);
extern void AL_InvPark(volatile Ip_t *IPARK, float sin_t, float cos_t);
#endif