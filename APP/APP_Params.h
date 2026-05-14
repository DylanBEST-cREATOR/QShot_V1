#ifndef  __APP_PARAMS_H
#define  __APP_PARAMS_H


/* ==================== 电流与电压标幺化全局常数 ==================== */
// 电流物理参数
#define ADC_VREF             3.3f
#define SAMPLING_RES         0.0005f
#define ADC_DATARANGE        4096           // 12位ADC转换物理量时，分母必须用2^12 = 4096
#define OPAMP_GAIN           40             


#define I_CAL_COEFF          (2640)        //q16格式  //（ADC_VREF/SAMPLING_RES*OPAMP_GAIN）* (65536/ADC_DATARANGE),用adc读数乘以这个系数=标幺化后的电流


#define I_BASE_Q16  (655360)      // 10A    

// 电压物理参数
#define V_BUS_MAX_PHYS       (62.7f)        // 3.3 * (1 + 18) = 62.7 V
#define VBUS_CAL_COEFF       (1003)         // 62.7 * 16 = 1003.2 ≈ 1003
#define V_BASE_Q16           (786432)      // 12.0 * 65536


//#define POLE_PAIRS 7   // 替换为你电机的实际极对数

/* ==================== 速度环标幺化参数 ==================== */
// 1. 速度计算系数 (针对 1ms 采样周期, 4096线编码器)
// 计算逻辑：RPM = (delta_counts / 4096) * 1000Hz * 60s
// RPM = delta_counts * (60000 / 4096) ≈ delta_counts * 14.6484
// 在 Q16 格式下： 系数 = (60000 / 4096) * 65536 = 960000
#define SPEED_CAL_COEFF      (192000)      

// 2. 速度限幅
#define MAX_SPEED_RPM        (1800)        // 物理极限 1800 RPM
#define SPEED_LIMIT_Q16      (117964800)   // 1800 * 65536 (Q16格式的目标上限)

// 3. 速度环输出限幅 (即允许的最大 Q 轴电流)
#define SPEED_PID_OUT_MAX  (98304)      // 1.5A * 65536

#endif
