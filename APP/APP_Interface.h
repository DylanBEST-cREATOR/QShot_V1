#ifndef __APP_INTERFACE_H
#define __APP_INTERFACE_H
#include "APP_Params.h"
#include "FOC_Qformat.h"
#include "stm32f4xx_hal.h"
typedef struct {
    // ==================== 1. 硬件偏置值 ====================
    int32_t offset_1;           // 通道 1 (A相) 偏置 (Q0 原始值)
    int32_t offset_2;           // 通道 2 (B相) 偏置 (Q0 原始值)
    int32_t offset_3;           // 通道 3 (C相) 偏置 (Q0 原始值，三电阻采样时使用)

} Current_Offset_t;


#endif
