#include "APP_Interface.h"


void MyDriver_Init(void){
//	DRV8301_Init();
 
}
void AI_ADCOffset_Config(void){
    // 启动从机 ADC2（双 ADC 模式下 Slave 需先准备好）
    HAL_ADCEx_InjectedStart(&hadc2);
    // 启动主机 ADC1 并开启中断
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    
    // 启动定时器和触发通道
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	 __HAL_TIM_MOE_DISABLE(&htim1); }



/**
 * @brief  偏置电压采集累加处理函数
 * @note   该函数需在 ADC 注入中断回调中调用，每次调用处理一个样本点
 * @param  pOffsets: 指向存储偏置结果的结构体指针
 */


uint8_t AI_OffsetCalibration_Process(Current_Sense_t* pOffsets) 
{
    static uint32_t internal_sum_b = 0;
    static uint32_t internal_sum_c = 0;
    static uint16_t internal_count = 0;
	
    
    // 1. 累加采样值
    internal_sum_b += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    internal_sum_c += HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    internal_count++;

    // 2. 检查是否达到采样目标
    if (internal_count >= 1024) 
    {
        // 计算平均值
//        pOffsets->offset_2 = (int32_t)(internal_sum_b >> 7);
//        pOffsets->offset_3 = (int32_t)(internal_sum_c >> 7);
//        pOffsets->offset_1 = 2048; // 默认中值
			    pOffsets->offset_2 = (int32_t)(internal_sum_b >> 10);
			    pOffsets->offset_3 = (int32_t)(internal_sum_c >> 10);

        // 重要：清零所有静态变量，为下次校准做准备
//        internal_sum_b = 0;
//        internal_sum_c = 0;
        internal_count = 0; 
        
        return 1; // 校准完成
    }

    return 0; // 校准正在进行中
}










