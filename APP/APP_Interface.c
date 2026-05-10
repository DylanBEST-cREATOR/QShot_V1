#include "APP_Interface.h"


void AI_ADCVoltageOffsets_Get(Current_Offset_t* ADC_VOLTAGE) {
    // 1. 初始化 32 位局部累加器（每次调用强制清零，绝对安全）
    uint32_t sum_offset_1 = 0; 
    uint32_t sum_offset_2 = 0;
    uint32_t sum_offset_3 = 0;

    for (uint8_t ReadCounts = 0; ReadCounts < 128; ReadCounts++) {
        // 2. 软件同时触发三个通道的 ADC 转换
//        ADC_SoftTrig(ADC, SOC0_TRIG);   // 触发 A 相 (通道 1)
//        ADC_SoftTrig(ADC, SOC1_TRIG);   // 触发 B 相 (通道 2)
//        ADC_SoftTrig(ADC, SOC2_TRIG);   // 触发 C 相 (通道 3)  <-- 新增
        
        // ⚠️ 硬件等待提示：
        // 如果你的 MCU 需要手动等待 EOC (End of Conversion) 标志，请在此处加上等待。
        // 例如：while(ADC_GetITStatus(ADC, ADC_IT_EOC2) == RESET); // 等待最后转换完成的通道
        
        // 3. 累加 12 位 ADC 原始寄存器值
//        sum_offset_1 += ADC->DR0; 
//        sum_offset_2 += ADC->DR1;
//        sum_offset_3 += ADC->DR2;       // <-- 新增
    }
    
    // 4. 右移 7 位 (相当于除以 128) 计算平均值，并存入你的新结构体
    ADC_VOLTAGE->offset_1 = (int32_t)(sum_offset_1 >> 7); 
    ADC_VOLTAGE->offset_2 = (int32_t)(sum_offset_2 >> 7);
    ADC_VOLTAGE->offset_3 = (int32_t)(sum_offset_3 >> 7);       // <-- 新增
    
    // 5. 停止 ADC 转换，清除转换结束中断标志位
//    ADC_StopOfConversion(ADC);
//    ADC_ClearSOCITPendingBit(ADC, ADC_IT_EOC2); // 确保清除了最后的转换通道标志
}




