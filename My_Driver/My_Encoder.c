#include "My_Encoder.h"

#if defined (USE_AS5600)
void ME_MyI2C_Init(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

// 简单的微秒延迟，根据你的 CPU 频率调整，这里假设 F4 系列 168MHz
void I2C_Delay(void) {
    for(volatile int i = 0; i < 3; i++); 
}



void I2C_Start(void) {
    SDA_H; SCL_H; I2C_Delay();
    SDA_L; I2C_Delay();
    SCL_L; I2C_Delay();
}

void I2C_Stop(void) {
    SDA_L; SCL_H; I2C_Delay();
    SDA_H; I2C_Delay();
}

uint8_t I2C_Wait_Ack(void) {
    uint8_t retry = 0;
    SDA_H; I2C_Delay();
    SCL_H; I2C_Delay();
    while (SDA_READ) {
        retry++;
        if (retry > 250) { I2C_Stop(); return 1; }
    }
    SCL_L; I2C_Delay();
    return 0;
}

void I2C_SendByte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        if (byte & 0x80) SDA_H; else SDA_L;
        I2C_Delay();
        SCL_H; I2C_Delay();
        SCL_L; I2C_Delay();
        byte <<= 1;
    }
}

uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t res = 0;
    SDA_H; 
    for (int i = 0; i < 8; i++) {
        SCL_H; I2C_Delay();
        res <<= 1;
        if (SDA_READ) res++;
        SCL_L; I2C_Delay();
    }
    if (ack) { SDA_L; } else { SDA_H; } // 发送应答或非应答
    SCL_H; I2C_Delay(); SCL_L; I2C_Delay();
    return res;
}





uint16_t _ME_AS5600_Read_Raw_Angle(void) {
    uint8_t high, low;

    I2C_Start();
    I2C_SendByte(AS5600_ADDR);       // 写地址
    I2C_Wait_Ack();
    I2C_SendByte(0x0E);              // 角度高字节寄存器地址
    I2C_Wait_Ack();
    
    I2C_Start();                     // 重启信号
    I2C_SendByte(AS5600_ADDR | 0x01);// 读地址
    I2C_Wait_Ack();
    high = I2C_ReadByte(1);          // 读高 8 位，发送 ACK
    low = I2C_ReadByte(0);           // 读低 8 位，发送 NACK
    I2C_Stop();

    return ((uint16_t)high << 8) | low;
}






extern uint16_t Global_AS5600_Raw_Angle; // 在 main 的 while(1) 里更新
extern uint16_t AS5600_Zero_Offset;      // 在 ROTOR_ALIGN 结束时记录
#endif


#define POLE_PAIRS 7   // 替换为你电机的实际极对数
//extern uint16_t MT6701_Zero_Offset;
//extern uint16_t MT6701_Zero_Offset;
extern uint16_t AS5600_Zero_Offset;
extern uint16_t Global_AS5600_Raw_Angle; 
/**
 * @brief 将 AS5600 的机械角度转换为 Q16 电角度
 * @return 范围在 [-65536, 65536] 之间的电角度
 */
 
 
 
/* 定义 CS 引脚的操作，方便阅读 */
//#define MT6701_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)
//#define MT6701_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)

/**
 * @brief  获取 MT6701 的原始 14 位角度值
 * @return uint16_t 0~16383 之间的角度值 (14-bit)
 */
//uint16_t MT6701_Get_Raw_Angle(void)
//{
//    uint8_t rx_data[3] = {0}; // 存储返回的 24 位数据
//    uint16_t angle_raw = 0;
//    extern SPI_HandleTypeDef hspi3; // 确保声明了 SPI3 句柄

//    // 1. 拉低 CS，开始通信
//    MT6701_CS_LOW();

//    // 2. 使用 HAL 库接收 3 个字节 (24 bits)
//    // MT6701 是全双工/半双工通用，直接 Receive 即可，它会产生时钟让传感器吐出数据
//    if (HAL_SPI_Receive(&hspi3, rx_data, 3, HAL_MAX_DELAY) == HAL_OK)
//    {
//        // 3. 拉高 CS，结束通信
//        MT6701_CS_HIGH();

//        /* 
//         * 数据解析说明：
//         * MT6701 输出 24 位：[23:10] 是角度 (14bit), [9:6] 是状态位, [5:0] 是 CRC
//         * rx_data[0] = Angle[13:6]
//         * rx_data[1] = Angle[5:0] + Status[3:2]
//         * rx_data[2] = Status[1:0] + CRC[5:0]
//         */
//        angle_raw = (rx_data[0] << 6) | (rx_data[1] >> 2);
//    }
//    else
//    {
//        // 通信失败处理
//        MT6701_CS_HIGH();
//        return 0; 
//    }

//    return angle_raw;
//}

/**
 * @brief 获取浮点数角度 (0.0 ~ 360.0)
 */
//float MT6701_Get_Angle_Degrees(void)
//{
//    uint16_t raw = MT6701_Get_Raw_Angle();
//    return (float)raw * 360.0f / 16384.0f;
//}
/**
 * @brief 获取 MT6701 的电角度 (Q16 格式: [-65536, 65536])
 * @return int32_t Q16 格式的角度值
 */
int32_t ME_Get_Electrical_Angle_Q16(void) {

    // 1. 获取当前最新机械角度

    int32_t raw_mech = Global_AS5600_Raw_Angle;



    // 2. 减去零位偏移

    int32_t mech_diff = raw_mech - AS5600_Zero_Offset;

    

    // 处理溢出：如果当前角度小于偏移量，加上一圈的总量 (4096)

    if (mech_diff < 0) {

        mech_diff += 4096;

    }



    // 3. 计算电角度 (未经标幺化的电角度值)

    int32_t elec_raw = (mech_diff * POLE_PAIRS) % 4096;



    // 4. 将 0~4095 的范围映射到 [-65536, 65536]

    // 比例关系：(elec_raw / 4096) * 131072 - 65536

    // 优化计算：(elec_raw * 32) - 65536，即左移 5 位

    int32_t q16_angle = (elec_raw << 5) - 65536;



    // 5. 限幅保护（虽然理论上不会超，但加上更安全）

    if (q16_angle > 65536)  q16_angle -= 131072;

    else if (q16_angle < -65536) q16_angle += 131072;



    // ==========================================

    // ⚠️ 极性反转开关 ⚠️

    // ==========================================

    // 在你进行正式闭环前，先运行一次开环拖动。

    // 如果电机实际转动方向与 AS5600 角度递增方向相反，

    // 请取消下方代码的注释，将角度取反：

    // q16_angle = -q16_angle;

    

    return q16_angle;

}