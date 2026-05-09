#ifndef __FOC_SVPWM_H
#define __FOC_SVPWM_H
typedef struct {
    float U_alpha;      // 输入 Alpha 轴电压
    float U_beta;       // 输入 Beta 轴电压
    float U_dc;         // 母线电压
    float T_pwm;        // PWM 周期（或设置为 1.0 用于输出占空比）
    
    
    float Duty_A;
    float Duty_B;
    float Duty_C;
} SVPWM_t;
extern void AL_SVPWM_Calculate(volatile SVPWM_t *SVPWM);
#endif
