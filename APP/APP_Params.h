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
/* 速度计算系数计算公式：
   RPM = (delta_counts / 4096) * 1000Hz * 60s
   RPM = delta_counts * 14.6484
   Q16系数 = 14.6484 * 65536 = 960000 
*/
#define SPEED_SAMPLING_FREQ  1000      // 速度环频率 1000Hz
#define SPEED_CAL_COEFF      960000    // Q16格式系数
#define SPEED_PID_OUT_MAX    65536
#endif
