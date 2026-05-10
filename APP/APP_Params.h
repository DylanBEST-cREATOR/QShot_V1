#ifndef  __APP_PARAMS_H
#define  __APP_PARAMS_H


/* ==================== 电流与电压标幺化全局常数 ==================== */
// 电流物理参数
#define ADC_VREF             3.3f
#define SAMPLING_RES         0.0005f
#define ADC_DATARANGE        4096           // 12位ADC转换物理量时，分母必须用2^12 = 4096
#define OPAMP_GAIN           20             


#define I_CAL_COEFF          (5280)        //q16格式  //（ADC_VREF/SAMPLING_RES*OPAMP_GAIN）* (65536/ADC_DATARANGE),用adc读数乘以这个系数=标幺化后的电流

// 2. 假设你的系统额定电流基准是 30.0 A
#define I_BASE_Q16           (1966080)      // 30.0 * 65536

// 电压物理参数
#define V_BUS_MAX_PHYS       (62.7f)        // 3.3 * (1 + 18) = 62.7 V
#define VBUS_CAL_COEFF       (1003)         // 62.7 * 16 = 1003.2 ≈ 1003
#define V_BASE_Q16           (1572864)      // 24.0 * 65536

#endif
