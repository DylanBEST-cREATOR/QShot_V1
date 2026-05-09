#ifndef __APP_INTERFACE_H
#define __APP_INTERFACE_H
#include "APP_Params.h"
#include "FOC_Qformat.h"
typedef struct {
    // ==================== 1. 硬件偏置值 ====================
    // 存储校准出来的原始 ADC 寄存器偏置值（例如 12 位 ADC 约在 2048 左右）
    // 保持原始整数，不需要定点化
    int32_t offset_1;           // 通道 1 (A相) 偏置
    int32_t offset_2;           // 通道 2 (B相) 偏置
	  int32_t offset_3;  

    // ==================== 2. 实时原始采样值 ====================
    // 每次在 ADC 中断中读取的最新原始寄存器数值
    int32_t adc_raw_1;          // 通道 1 实时原始值
    int32_t adc_raw_2;          // 通道 2 实时原始值
	  int32_t adc_raw_3; 

    // ==================== 3. 标幺化转换系数 (Q16) ====================
    // 用于将 (adc_raw - offset) 的差值，一步转换成 Q16 标幺化电流
    q16_t adc_to_pu_gain;       

    // ==================== 4. 最终标幺化相电流输出 (Q16) ====================
    // 对应定点数 [-65536, 65536]
    q16_t Ia;                   // A 相电流标幺值
    q16_t Ib;                   // B 相电流标幺值
    q16_t Ic;                   // C 相电流标幺值 (由 Ia + Ib + Ic = 0 算出)
} Current_Sense_t;


#endif