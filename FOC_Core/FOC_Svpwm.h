#ifndef __FOC_SVPWM_H
#define __FOC_SVPWM_H
#include "FOC_Qformat.h"
#include "APP_Params.h"
#include <stdbool.h>
#include <stddef.h>
#define Q16_SQRT3   (113512)   // sqrt(3) 的 Q16 定点数 (1.7320508 * 65536)

typedef struct {
    q16_t U_alpha;      // 输入 Alpha 轴电压 (Q16 标幺值 / 物理值)
    q16_t U_beta;       // 输入 Beta 轴电压 (Q16 标幺值 / 物理值)
    q16_t U_dc;         // 母线电压 (Q16 物理值，必须与 U_alpha/beta 同标定)
    uint32_t T_pwm;     // PWM 周期（直接传入定时器的自动重装载值 ARR，例如 3500）
    
    uint32_t Duty_A;    // 输出 A 相占空比定时器寄存器值 (0 ~ T_pwm)
    uint32_t Duty_B;    // 输出 B 相占空比定时器寄存器值 (0 ~ T_pwm)
    uint32_t Duty_C;    // 输出 C 相占空比定时器寄存器值 (0 ~ T_pwm)
} SVPWM_t;



typedef struct {
    // ==================== 1. 硬件偏置值 ====================
    int32_t offset_1;           // 通道 1 (A相) 偏置 (Q0 原始值)
    int32_t offset_2;           // 通道 2 (B相) 偏置 (Q0 原始值)
    int32_t offset_3;           // 通道 3 (C相) 偏置 (Q0 原始值，三电阻采样时使用)

    // ==================== 2. 实时原始采样值 ====================
    int32_t adc_raw_1;          // 通道 1 实时原始值
    int32_t adc_raw_2;          // 通道 2 实时原始值
    int32_t adc_raw_3;          // 通道 3 实时原始值

    // ==================== 3. 最终标幺化相电流输出 (Q16) ====================
    q16_t Ia;                   // A 相电流标幺值 (1.0 PU = 65536)
    q16_t Ib;                   // B 相电流标幺值 (1.0 PU = 65536)
    q16_t Ic;                   // C 相电流标幺值 (1.0 PU = 65536)
} Current_Sense_t;



//static __inline q16_t FS_Get_Electrical_Angle(uint16_t raw_angle, uint16_t offset_angle, uint8_t pole_pairs, uint8_t reverse) {
//    // 1. 计算机械角度偏差 (处理 0-4095 回环)
//    int32_t mech_diff = (int32_t)raw_angle - (int32_t)offset_angle;
//    if (mech_diff < 0) {
//        mech_diff += 4096;
//    }

//    // 2. 计算电角度并取模 (0-4095)
//    int32_t elec_angle_raw = (mech_diff * pole_pairs) % 4096;

//    // 3. 映射到 Q16 格式 [-65536, 65536] (即 -PI 到 PI)
//    // (elec_angle_raw << 5) 将 4096 放大到 131072，减去一半得到对称区间
//    int32_t angle_q16 = (elec_angle_raw << 5) - 65536;

//    // 4. 根据 reverse 参数处理极性
//    if (reverse) {
//        return -angle_q16;
//    } else {
//        return angle_q16;
//    }
//}

static __inline q16_t FS_Get_Electrical_Angle(uint16_t raw_angle, uint16_t offset_angle, uint8_t pole_pairs, uint8_t reverse) {
    // 1. 计算机械角度偏差 (0-4095)
    int32_t mech_diff = (int32_t)raw_angle - (int32_t)offset_angle;
    if (mech_diff < 0) mech_diff += 4096;

    // 2. 计算电角度 (0-4095)
    int32_t elec_angle_raw = (mech_diff * pole_pairs) & 4095; // 使用 &4095 代替 %4096 更快

    // 3. 映射到 Q16 格式 [0, 131071] (对应 0 到 2*PI)
    // 直接左移 5 位，0 -> 0, 4095 -> 131040
    int32_t angle_q16 = (elec_angle_raw << 5);

    // 4. 将 [0, 2*PI] 转换为 [-PI, PI] 格式以适配 FM_fastSinCos
    if (angle_q16 > 65536) {
        angle_q16 -= 131072;
    }

    // 5. 极性处理
    if (reverse) {
        return (q16_t)(-angle_q16);
    } else {
        return (q16_t)angle_q16;
    }
}


/**
 * @brief  将实时 ADC 电流读数转换为 Q16 标幺化电流 (双电阻采样版)
 * @param  current: 电流结构体指针
 * @note   完美契合母线标幺化函数，采用 4096 绝对整除逻辑，零浮点开销
 */
static __inline void FS_ADCValueToCurrent(Current_Sense_t *current) {
    // 1. 减去校准好的硬件中点偏置
    int32_t diff_b = current->adc_raw_2 - current->offset_2;
    int32_t diff_c = current->adc_raw_3 - current->offset_3;
    
    // 2. 利用基尔霍夫定律在原始层计算 A 相，防止误差累积和三相不平衡
    int32_t diff_a = -(diff_b + diff_c);

    // 3. 一步到位转换至 Q16 标幺化电流
    // 说明：diff(整数) * I_SCALE_FACTOR(Q31) 结果为 Q31 格式。
    // 我们需要 Q16 格式输出，因此需要右移 31 - 16 = 15 位。
    // 右移 15 位前，加上 1 << 14 (即 16384) 用于实现精准的四舍五入。
    
    int64_t temp_ia = (int64_t)diff_a * I_SCALE_FACTOR;
    current->Ia = (q16_t)((temp_ia + 16384) >> 15);

    int64_t temp_ib = (int64_t)diff_b * I_SCALE_FACTOR;
    current->Ib = (q16_t)((temp_ib + 16384) >> 15);

    int64_t temp_ic = (int64_t)diff_c * I_SCALE_FACTOR;
    current->Ic = (q16_t)((temp_ic + 16384) >> 15);
}



static __inline q16_t FS_Convert_Vbus_To_PU(uint16_t adc_raw_val) {
    // 第一步：将无单位的 ADC 读数翻译为 Q16 格式的“物理实际电压” (单位：V)
    // 物理电压 = adc_raw_val * 1003
    int32_t v_phys_q16 = (int32_t)adc_raw_val * VBUS_CAL_COEFF;

    // 第二步：将物理电压标幺化，除以 24V 的电压基准
    // 数学公式：U_dc_PU = (v_phys_q16 * 65536) / V_BASE_Q16
    // ⚠️ 注意：两个 Q16 的数直接相除会退化为普通的小数（Q0），
    // 想要让结果继续保持 Q16 格式，必须在除法前先把分子左移 16 位（乘以 65536）。
    // 为了防止左移 16 位后数据溢出，我们强制转换为 64 位整型 (int64_t) 进行计算。
    int64_t temp_pu = ((int64_t)v_phys_q16 << 16) / V_BASE_Q16;

    // 第三步：将 64 位结果安全裁剪回 32 位的 q16_t 并返回
    return (q16_t)temp_pu;
}

static __inline uint32_t SVPWM_ClampCCR_i64(int64_t val, uint32_t min_val, uint32_t max_val)
{
    if (val < (int64_t)min_val) return min_val;
    if (val > (int64_t)max_val) return max_val;
    return (uint32_t)val;
}

static __inline int32_t SVPWM_AbsI32(int32_t x)
{
    return (x < 0) ? -x : x;
}

/**
 * @brief  将三相 CCR 限制在安全窗口内
 * @note   推荐使用“围绕中心点缩放”的方式，而不是直接硬裁剪。
 */
static __inline void SVPWM_LimitCCRWindow(uint32_t *a,
                                          uint32_t *b,
                                          uint32_t *c,
                                          uint32_t duty_mid,
                                          uint32_t ccr_min,
                                          uint32_t ccr_max)
{
#if SVPWM_USE_CENTER_SCALE_LIMIT

    int32_t da = (int32_t)(*a) - (int32_t)duty_mid;
    int32_t db = (int32_t)(*b) - (int32_t)duty_mid;
    int32_t dc = (int32_t)(*c) - (int32_t)duty_mid;

    int32_t abs_a = SVPWM_AbsI32(da);
    int32_t abs_b = SVPWM_AbsI32(db);
    int32_t abs_c = SVPWM_AbsI32(dc);

    int32_t max_abs = abs_a;
    if (abs_b > max_abs) max_abs = abs_b;
    if (abs_c > max_abs) max_abs = abs_c;

    int32_t allow_pos = (int32_t)ccr_max - (int32_t)duty_mid;
    int32_t allow_neg = (int32_t)duty_mid - (int32_t)ccr_min;

    int32_t allow = (allow_pos < allow_neg) ? allow_pos : allow_neg;

    if (allow < 0) allow = 0;

    /*
     * 如果三相偏离中心点的最大幅度超过允许范围，
     * 则按比例压缩三相偏移量。
     */
    if (max_abs > allow && max_abs > 0)
    {
        da = (int32_t)(((int64_t)da * allow) / max_abs);
        db = (int32_t)(((int64_t)db * allow) / max_abs);
        dc = (int32_t)(((int64_t)dc * allow) / max_abs);

        *a = SVPWM_ClampCCR_i64((int64_t)duty_mid + da, ccr_min, ccr_max);
        *b = SVPWM_ClampCCR_i64((int64_t)duty_mid + db, ccr_min, ccr_max);
        *c = SVPWM_ClampCCR_i64((int64_t)duty_mid + dc, ccr_min, ccr_max);
    }
    else
    {
        *a = SVPWM_ClampCCR_i64(*a, ccr_min, ccr_max);
        *b = SVPWM_ClampCCR_i64(*b, ccr_min, ccr_max);
        *c = SVPWM_ClampCCR_i64(*c, ccr_min, ccr_max);
    }

#else

    /*
     * 简单硬裁剪版本。
     */
    *a = SVPWM_ClampCCR_i64(*a, ccr_min, ccr_max);
    *b = SVPWM_ClampCCR_i64(*b, ccr_min, ccr_max);
    *c = SVPWM_ClampCCR_i64(*c, ccr_min, ccr_max);

#endif
}



/**
 * @brief  SVPWM 占空比计算 (修正后的 Q16 高精度版)
 * @param  SVPWM_Compnents: SVPWM 结构体指针
 */
/**
 * @brief  SVPWM 占空比计算，适配中心对齐 PWM + PWM Mode 2 的现有工程版本
 *
 * @note
 *  你的当前工程约定：
 *  1. TIM1 为中心对齐模式；
 *  2. TIM1 ARR = 4200；
 *  3. SVPWM_Compnents->T_pwm = 8400 = 2 * ARR；
 *  4. 输出 Duty_A/B/C 是 TIM1 CCR 值；
 *  5. CCR 有效范围是 0 ~ ARR，即 0 ~ T_pwm / 2；
 *  6. 零电压时 CCR 应为 ARR / 2，即 T_pwm / 4 = 2100。
 *
 * @param SVPWM_Compnents SVPWM 结构体指针
 */
static __inline void FS_SVPWM_Calculate(SVPWM_t *SVPWM_Compnents)
{
    if (SVPWM_Compnents == NULL) {
        return;
    }

    /* ==================== 1. 缓存输入 ==================== */
    int32_t u_alpha = SVPWM_Compnents->U_alpha;
    int32_t u_beta  = SVPWM_Compnents->U_beta;
    int32_t u_dc    = SVPWM_Compnents->U_dc;
    uint32_t t_pwm  = SVPWM_Compnents->T_pwm;

    /*
     * 当前工程：
     * t_pwm = 2 * ARR
     * ccr_hw_max = ARR
     * duty_mid = ARR / 2
     */
    if (t_pwm < 4) {
        SVPWM_Compnents->Duty_A = 0;
        SVPWM_Compnents->Duty_B = 0;
        SVPWM_Compnents->Duty_C = 0;
        return;
    }

    uint32_t ccr_hw_max = t_pwm >> 1;  // 例如 8400 / 2 = 4200
    uint32_t duty_mid   = t_pwm >> 2;  // 例如 8400 / 4 = 2100

    /*
     * 安全窗口。
     * 如果宏定义不合理，则退回到完整 CCR 范围。
     */
    uint32_t ccr_safe_min = SVPWM_CCR_MIN_LIMIT;
    uint32_t ccr_safe_max = SVPWM_CCR_MAX_LIMIT;

    if (ccr_safe_min >= ccr_safe_max || ccr_safe_max > ccr_hw_max) {
        ccr_safe_min = 0;
        ccr_safe_max = ccr_hw_max;
    }

    /*
     * 母线电压保护。
     * u_dc 异常时直接输出零电压中心占空比。
     */
    if (u_dc < 6554) {  // 0.1 PU 以下认为无效
        SVPWM_Compnents->Duty_A = duty_mid;
        SVPWM_Compnents->Duty_B = duty_mid;
        SVPWM_Compnents->Duty_C = duty_mid;
        return;
    }

    /* ==================== 2. 计算三相判别量 ==================== */
    int32_t v1 = u_beta;

    int32_t sqrt3_u_alpha =
        (int32_t)(((int64_t)u_alpha * Q16_SQRT3) >> 16);

    int32_t v2 = (sqrt3_u_alpha - u_beta) >> 1;
    int32_t v3 = (-sqrt3_u_alpha - u_beta) >> 1;

    /* ==================== 3. 扇区判别 ==================== */
    int32_t sector = 0;

    if (v1 > 0) sector += 1;
    if (v2 > 0) sector += 2;
    if (v3 > 0) sector += 4;

    /*
     * 零矢量处理。
     */
    if (sector == 0 || sector == 7) {
        SVPWM_Compnents->Duty_A = duty_mid;
        SVPWM_Compnents->Duty_B = duty_mid;
        SVPWM_Compnents->Duty_C = duty_mid;
        return;
    }

    /* ==================== 4. 预计算系数 K ==================== */
    /*
     * K = sqrt(3) * T_pwm / Udc
     */
    int64_t K_num = (int64_t)Q16_SQRT3 * (int64_t)t_pwm;
    int64_t K = (K_num << 16) / (int64_t)u_dc;

    /* ==================== 5. 计算 T1/T2 ==================== */
    int64_t T1 = 0;
    int64_t T2 = 0;

    switch (sector)
    {
        case 3:     // Sector 1
            T1 = ((int64_t)v2 * K) >> 16;
            T2 = ((int64_t)v1 * K) >> 16;
            break;

        case 1:     // Sector 2
            T1 = ((int64_t)(-v2) * K) >> 16;
            T2 = ((int64_t)(-v3) * K) >> 16;
            break;

        case 5:     // Sector 3
            T1 = ((int64_t)v1 * K) >> 16;
            T2 = ((int64_t)v3 * K) >> 16;
            break;

        case 4:     // Sector 4
            T1 = ((int64_t)(-v1) * K) >> 16;
            T2 = ((int64_t)(-v2) * K) >> 16;
            break;

        case 6:     // Sector 5
            T1 = ((int64_t)v3 * K) >> 16;
            T2 = ((int64_t)v2 * K) >> 16;
            break;

        case 2:     // Sector 6
            T1 = ((int64_t)(-v3) * K) >> 16;
            T2 = ((int64_t)(-v1) * K) >> 16;
            break;

        default:
            SVPWM_Compnents->Duty_A = duty_mid;
            SVPWM_Compnents->Duty_B = duty_mid;
            SVPWM_Compnents->Duty_C = duty_mid;
            return;
    }

    if (T1 < 0) T1 = 0;
    if (T2 < 0) T2 = 0;

    /* ==================== 6. 过调制处理 ==================== */
    int64_t t_pwm_q16 = ((int64_t)t_pwm) << 16;
    int64_t Tsum = T1 + T2;

    if (Tsum > t_pwm_q16 && Tsum > 0)
    {
        T1 = (T1 * t_pwm_q16) / Tsum;
        T2 = (T2 * t_pwm_q16) / Tsum;
    }

    /* ==================== 7. 计算七段式 PWM 时间点 ==================== */
    int64_t Ta = (t_pwm_q16 - T1 - T2) >> 2;
    int64_t Tb = Ta + (T1 >> 1);
    int64_t Tc = Tb + (T2 >> 1);

    /* ==================== 8. Q16 转 CCR，先做硬件范围限幅 ==================== */
    int64_t duty_a_i64 = (Ta + 32768) >> 16;
    int64_t duty_b_i64 = (Tb + 32768) >> 16;
    int64_t duty_c_i64 = (Tc + 32768) >> 16;

    uint32_t duty_a = SVPWM_ClampCCR_i64(duty_a_i64, 0, ccr_hw_max);
    uint32_t duty_b = SVPWM_ClampCCR_i64(duty_b_i64, 0, ccr_hw_max);
    uint32_t duty_c = SVPWM_ClampCCR_i64(duty_c_i64, 0, ccr_hw_max);

    /* ==================== 9. 扇区映射 ==================== */
    uint32_t phase_a = duty_mid;
    uint32_t phase_b = duty_mid;
    uint32_t phase_c = duty_mid;

    switch (sector)
    {
        case 3:     // Sector 1
            phase_a = duty_a;
            phase_b = duty_b;
            phase_c = duty_c;
            break;

        case 1:     // Sector 2
            phase_a = duty_b;
            phase_b = duty_a;
            phase_c = duty_c;
            break;

        case 5:     // Sector 3
            phase_a = duty_c;
            phase_b = duty_a;
            phase_c = duty_b;
            break;

        case 4:     // Sector 4
            phase_a = duty_c;
            phase_b = duty_b;
            phase_c = duty_a;
            break;

        case 6:     // Sector 5
            phase_a = duty_b;
            phase_b = duty_c;
            phase_c = duty_a;
            break;

        case 2:     // Sector 6
            phase_a = duty_a;
            phase_b = duty_c;
            phase_c = duty_b;
            break;

        default:
            phase_a = duty_mid;
            phase_b = duty_mid;
            phase_c = duty_mid;
            break;
    }

    /* ==================== 10. 直接在 SVPWM 内做 CCR 安全窗口限制 ==================== */
    SVPWM_LimitCCRWindow(&phase_a,
                         &phase_b,
                         &phase_c,
                         duty_mid,
                         ccr_safe_min,
                         ccr_safe_max);

    /* ==================== 11. 写回输出 ==================== */
    SVPWM_Compnents->Duty_A = phase_a;
    SVPWM_Compnents->Duty_B = phase_b;
    SVPWM_Compnents->Duty_C = phase_c;
}


#endif
