#include "APP_Interface.h"


void AI_ADCVoltageOffsets_Get(Current_Offset_t* ADC_VOLTAGE) {
    uint32_t sum_offset_1 = 0; 
    uint32_t sum_offset_2 = 0;
    uint32_t sum_offset_3 = 0;

    for (uint8_t ReadCounts = 0; ReadCounts < 128; ReadCounts++) {
//        sum_offset_1 += ADC1->DR0; 
//        sum_offset_2 += ADC1->DR1;
//       sum_offset_3 += ADC->DR2;      
    }
    
    // 4. 右移 7 位 (相当于除以 128) 计算平均值，并存入你的新结构体
    ADC_VOLTAGE->offset_1 = (int32_t)(sum_offset_1 >> 7); 
    ADC_VOLTAGE->offset_2 = (int32_t)(sum_offset_2 >> 7);
    ADC_VOLTAGE->offset_3 = (int32_t)(sum_offset_3 >> 7);      
    
}




