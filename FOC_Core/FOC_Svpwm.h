#ifndef __FOC_SVPWM_H
#define __FOC_SVPWM_H
#include "FOC_Qformat.h"
#include "APP_Params.h"

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


/**
 * @brief  将实时 ADC 电流读数转换为 Q16 标幺化电流 (双电阻采样版)
 * @param  current: 电流结构体指针
 * @note   完美契合母线标幺化函数，采用 4096 绝对整除逻辑，零浮点开销
 */
static __inline void FS_ADCValueToCurrent(Current_Sense_t *current) {
    // 1. 减去校准好的硬件中点偏置，得到纯净的 ADC 偏差刻度值
    int32_t diff_a = current->adc_raw_1 - current->offset_1;
    int32_t diff_b = current->adc_raw_2 - current->offset_2;

    // ==================== A 相电流转换 ====================
    // 第一步：将 ADC 刻度差值翻译为 Q16 格式的“物理实际电流” (单位：A)
    int32_t ia_phys_q16 = diff_a * I_CAL_COEFF;


    int64_t temp_ia_pu = ((int64_t)ia_phys_q16 << 16) / I_BASE_Q16;


    current->Ia = -(q16_t)temp_ia_pu;   //Q16格式的标幺定点化电流

    // ==================== B 相电流转换 ====================
    // 第一步：翻译物理实际电流
    int32_t ib_phys_q16 = diff_b * I_CAL_COEFF;

    // 第二步：标幺化
    int64_t temp_ib_pu = ((int64_t)ib_phys_q16 << 16) / I_BASE_Q16;

    // 第三步：写入
    current->Ib = -(q16_t)temp_ib_pu;

    // ==================== C 相电流计算 ====================
    // 基尔霍夫定律：Ic = -(Ia + Ib)
    current->Ic = -(current->Ia + current->Ib);
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


/**
 * @brief  SVPWM 占空比计算 (Q16 定点标幺化生产版)
 * @param  SVPWM_Compnents: SVPWM 结构体指针
 * @note   输入电压为 Q16，输出占空比直接为定时器寄存器 Tick 值，无任何浮点计算
 */

static __inline void FS_SVPWM_Calculate(SVPWM_t *SVPWM_Compnents) {
    // 缓存输入变量到 CPU 寄存器，规避指针寻址开销
    int32_t u_alpha = SVPWM_Compnents->U_alpha;
    int32_t u_beta  = SVPWM_Compnents->U_beta;
    int32_t u_dc    = SVPWM_Compnents->U_dc;
    uint32_t t_pwm  = SVPWM_Compnents->T_pwm;

    // ✅ 修正安全下限：设为 0.25 PU (对应 24V 基准下的 6.0V 物理电压，定点数 16384)
    // 既能防止除零，又能彻底断绝 K 的 32 位有符号整型溢出风险
    if (u_dc < 16384) {
        u_dc = 16384; 
    }

    // ==================== 1. 计算三相电压分量 v1, v2, v3 ====================
    int32_t v1 = u_beta;
    int32_t sqrt3_u_alpha = (int32_t)(((int64_t)u_alpha * Q16_SQRT3) >> 16);
    int32_t v2 = (sqrt3_u_alpha - u_beta) >> 1;
    int32_t v3 = (-sqrt3_u_alpha - u_beta) >> 1;

    // ==================== 2. 扇区判别 (Sector Section) ====================
    int32_t sector = 0;
    if (v1 > 0) sector += 1;
    if (v2 > 0) sector += 2;
    if (v3 > 0) sector += 4;

    // ==================== 3. 预计算定点系数 K ====================
    int64_t K_num = (int64_t)Q16_SQRT3 * t_pwm;
    int32_t K = (int32_t)((K_num << 16) / u_dc);     //若udc过小可能产生溢出

    // ==================== 4. 计算矢量作用时间 T1, T2 (Q16 格式 ticks) ====================
    int32_t T1, T2;
    switch (sector) {
        case 3: 
            T1 = (int32_t)(((int64_t)v2 * K) >> 16); 
            T2 = (int32_t)(((int64_t)v1 * K) >> 16); 
            break;   // Sector 1
        case 1: 
            T1 = (int32_t)(((int64_t)-v2 * K) >> 16); 
            T2 = (int32_t)(((int64_t)-v3 * K) >> 16); 
            break;   // Sector 2
        case 5: 
            T1 = (int32_t)(((int64_t)v1 * K) >> 16); 
            T2 = (int32_t)(((int64_t)v3 * K) >> 16); 
            break;   // Sector 3
        case 4: 
            T1 = (int32_t)(((int64_t)-v1 * K) >> 16); 
            T2 = (int32_t)(((int64_t)-v2 * K) >> 16); 
            break;   // Sector 4
        case 6: 
            T1 = (int32_t)(((int64_t)v3 * K) >> 16); 
            T2 = (int32_t)(((int64_t)v2 * K) >> 16); 
            break;   // Sector 5
        case 2: 
            T1 = (int32_t)(((int64_t)-v3 * K) >> 16); 
            T2 = (int32_t)(((int64_t)-v1 * K) >> 16); 
            break;   // Sector 6
        default: 
            T1 = 0; 
            T2 = 0; 
            break;
    }

    // ==================== 5. 过调制处理 (Over-modulation) ====================
    int32_t Tsum = T1 + T2;
    int32_t t_pwm_q16 = (int32_t)t_pwm << 16; 

    if (Tsum > t_pwm_q16) {
        T1 = (int32_t)(((int64_t)T1 * t_pwm_q16) / Tsum);
        T2 = (int32_t)(((int64_t)T2 * t_pwm_q16) / Tsum);
    }

    // ==================== 6. 计算三个半桥的 Duty 切换点 (Q16 格式 ticks) ====================
    int32_t Ta = (t_pwm_q16 - T1 - T2) >> 2;
    int32_t Tb = Ta + (T1 >> 1);
    int32_t Tc = Tb + (T2 >> 1);

    // ==================== 7. 将 Q16 格式 ticks 四舍五入还原为 Q0 格式定时器寄存器值 ====================
    uint32_t duty_a_raw = (uint32_t)((Ta + 32768) >> 16);
    uint32_t duty_b_raw = (uint32_t)((Tb + 32768) >> 16);
    uint32_t duty_c_raw = (uint32_t)((Tc + 32768) >> 16);

    // ==================== 8. 扇区占空比分配写入 ====================
    switch (sector) {
        case 3: 
            SVPWM_Compnents->Duty_A = duty_a_raw; 
            SVPWM_Compnents->Duty_B = duty_b_raw; 
            SVPWM_Compnents->Duty_C = duty_c_raw; 
            break;
        case 1: 
            SVPWM_Compnents->Duty_A = duty_b_raw; 
            SVPWM_Compnents->Duty_B = duty_a_raw; 
            SVPWM_Compnents->Duty_C = duty_c_raw; 
            break;
        case 5: 
            SVPWM_Compnents->Duty_A = duty_c_raw; 
            SVPWM_Compnents->Duty_B = duty_a_raw; 
            SVPWM_Compnents->Duty_C = duty_b_raw; 
            break;
        case 4: 
            SVPWM_Compnents->Duty_A = duty_c_raw; 
            SVPWM_Compnents->Duty_B = duty_b_raw; 
            SVPWM_Compnents->Duty_C = duty_a_raw; 
            break;
        case 6: 
            SVPWM_Compnents->Duty_A = duty_b_raw; 
            SVPWM_Compnents->Duty_B = duty_c_raw; 
            SVPWM_Compnents->Duty_C = duty_a_raw; 
            break; 
        case 2: 
            SVPWM_Compnents->Duty_A = duty_a_raw; 
            SVPWM_Compnents->Duty_B = duty_c_raw; 
            SVPWM_Compnents->Duty_C = duty_b_raw; 
            break;
        default:
            SVPWM_Compnents->Duty_A = 0;
            SVPWM_Compnents->Duty_B = 0;
            SVPWM_Compnents->Duty_C = 0;
            break;
    }
}
#endif
