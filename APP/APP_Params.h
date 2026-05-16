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
#endif