#include "APP_Interface.h"


void AI_ADCVoltageOffsets_Get(Current_Sense_t* ADC_VOLTAGE) {
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


void AI_ADCValueToCurrent(Current_Sense_t *current) {
    // 1. 读取最新的硬件 ADC 寄存器原始值（请根据你实际的寄存器修改 ADC->DR0 等）
//    current->adc_raw_1 = ADC->DR0; // A相
//    current->adc_raw_2 = ADC->DR1; // B相

    // 2. 减去校准好的硬件中点偏置，得到纯净的 ADC 偏差刻度值 (Q0 格式整数)
    int32_t diff_a = current->adc_raw_1 - current->offset_1;
    int32_t diff_b = current->adc_raw_2 - current->offset_2;

    // 3. 计算 A 相和 B 相的标幺化 Q16 电流
    // 算法解析：
    // a. (int64_t)diff_a * current->adc_to_pu_gain 
    //    将 32 位整数与 Q16 增益相乘。因为增益值高达 11537153，相乘极易溢出，
    //    所以强制转为 64 位整型（int64_t）进行计算，确保绝对安全。
    // b. >> 16
    //    右移 16 位，消去增益系数本身多带的 65536 倍，还原为标准的 Q16 格式。
    // c. 负号 '-' 
    //    保留你原本电路反相运放特性的负号。
    current->Ia = -(q16_t)(((int64_t)diff_a * current->adc_to_pu_gain) >> 16);
    current->Ib = -(q16_t)(((int64_t)diff_b * current->adc_to_pu_gain) >> 16);

    // 4. 根据基尔霍夫电流定律计算 C 相电流：Ia + Ib + Ic = 0  =>  Ic = -(Ia + Ib)
    // 直接进行 Q16 级别的加减法，没有任何精度损失
    current->Ic = -(current->Ia + current->Ib);
}


