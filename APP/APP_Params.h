#ifndef  __APP_PARAMS_H
#define  __APP_PARAMS_H


/* ==================== 电流与电压标幺化全局常数 ==================== */
/* ==================== 电流与电压标幺化全局常数 ==================== */
#define ADC_VREF              3.3f
#define SAMPLING_RES          0.0005f
#define ADC_DATARANGE         4096 
#define OPAMP_GAIN            40              // 已经修改为 40 倍

// 基准电流为 10A
#define I_BASE                10.0f 

// 一步到位标幺化系数（Q31格式）
// 3.3 / (0.0005 * 40 * 4096 * 10) * 2147483648 = 8650738
#define I_SCALE_FACTOR        (8650738LL)     // 更新此处的系数
#define CURRENT_PID_OUT_MAX    37800

// 电压物理参数
#define V_BUS_MAX_PHYS       (62.7f)        // 3.3 * (1 + 18) = 62.7 V
#define VBUS_CAL_COEFF       (1003)         // 62.7 * 16 = 1003.2 ≈ 1003
#define V_BASE_Q16           (786432)      // 24.0 


/* ==================== 速度环标幺化配置 (Base = 1800 RPM) ==================== */
#define MAX_SPEED_RPM        1800.0f       // 标幺化基准转速
#define SPEED_SAMPLING_FREQ  1000          // 采样频率 1kHz

// 新系数 = (960000 / 1800) ≈ 533
// 计算逻辑：delta * (14.648 / 1800) * 65536
#define SPEED_CAL_COEFF      533           

// 速度环输出限幅 (Iq指令标幺值)
// 假设你希望最大电流限制在 I_base 的 15%，则为 65536 * 0.15 ≈ 9830
#define SPEED_PID_OUT_MAX    9830          
#define SPEED_PID_OUT_MIN    -9830



#define TIM1_ARR_TICKS              4200U
#define SVPWM_TPWM_TICKS            8400U

/*
 * 低侧采样安全边界。
 *
 * 你现在 CCR4 = 1，触发非常靠近波谷。
 * 死区 150ns 约等于 25 ticks。
 * ADC 15 cycles 约等于 120 ticks。
 * 再加上 MOSFET、DRV8301 CSA、ADC 输入稳定裕量，
 * 推荐先用 360。
 */
#define SVPWM_CCR_MIN_LIMIT         360U

/*
 * 是否对上边界也做限制。
 * 对称限制可以减少极端调制带来的非线性。
 */
#define SVPWM_CCR_MAX_LIMIT         (TIM1_ARR_TICKS - SVPWM_CCR_MIN_LIMIT)

/*
 * 是否使用按中心缩放的限制方式。
 * 1：推荐，保持三相波形比例，只压缩调制幅度
 * 0：直接硬裁剪，更简单但失真更明显
 */
#define SVPWM_USE_CENTER_SCALE_LIMIT  1

#endif