#include "FOC_Svpwm.h"
void FS_SVPWM_Calculate(SVPWM_t *SVPWM_Compnents) {
    const float sqrt3 = 1.7320508f;
    const float inv_sqrt3 = 0.57735027f;
    
    float v1 = SVPWM_Compnents->U_beta;
    // 使用乘法代替除法
    float v2 = (sqrt3 * SVPWM_Compnents->U_alpha - SVPWM_Compnents->U_beta) * 0.5f;
    float v3 = (-sqrt3 * SVPWM_Compnents->U_alpha - SVPWM_Compnents->U_beta) * 0.5f;

    int sector = 0;
    if (v1 > 0.0f) sector += 1;
    if (v2 > 0.0f) sector += 2;
    if (v3 > 0.0f) sector += 4;

    // 预计算 K，减少后续乘法压力
    // K = sqrt3 * T_pwm / U_dc
    float K = (1.7320508f * SVPWM_Compnents->T_pwm) / SVPWM_Compnents->U_dc; 

    float T1, T2;
    switch (sector) {
        case 3: T1 = v2 * K; T2 = v1 * K; break;   // Sector 1
        case 1: T1 = -v2 * K; T2 = -v3 * K; break; // Sector 2
        case 5: T1 = v1 * K; T2 = v3 * K; break;   // Sector 3
        case 4: T1 = -v1 * K; T2 = -v2 * K; break; // Sector 4
        case 6: T1 = v3 * K; T2 = v2 * K; break;   // Sector 5
        case 2: T1 = -v3 * K; T2 = -v1 * K; break; // Sector 6
        default: T1 = 0; T2 = 0; break;
    }

    // 过调制处理
    float Tsum = T1 + T2;
    if (Tsum > SVPWM_Compnents->T_pwm) {
        float inv_Tsum = SVPWM_Compnents->T_pwm / Tsum;
        T1 *= inv_Tsum;
        T2 *= inv_Tsum;
    }

    // 计算 Duty，全部改为乘 0.25f 和 0.5f
    float Ta = (SVPWM_Compnents->T_pwm - T1 - T2) * 0.25f;
    float Tb = Ta + T1 * 0.5f;
    float Tc = Tb + T2 * 0.5f;

    switch (sector) {
        case 3: SVPWM_Compnents->Duty_A = Ta; SVPWM_Compnents->Duty_B = Tb; SVPWM_Compnents->Duty_C = Tc; break;
        case 1: SVPWM_Compnents->Duty_A = Tb; SVPWM_Compnents->Duty_B = Ta; SVPWM_Compnents->Duty_C = Tc; break;
        case 5: SVPWM_Compnents->Duty_A = Tc; SVPWM_Compnents->Duty_B = Ta; SVPWM_Compnents->Duty_C = Tb; break;
        case 4: SVPWM_Compnents->Duty_A = Tc; SVPWM_Compnents->Duty_B = Tb; SVPWM_Compnents->Duty_C = Ta; break;
        case 6: SVPWM_Compnents->Duty_A = Tb; SVPWM_Compnents->Duty_B = Tc; SVPWM_Compnents->Duty_C = Ta; break; 
        case 2: SVPWM_Compnents->Duty_A = Ta; SVPWM_Compnents->Duty_B = Tc; SVPWM_Compnents->Duty_C = Tb; break;
    }
}
