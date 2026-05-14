#ifndef __MY_ENCODER_H
#define __MY_ENCODER_H
#include "stm32f4xx_hal.h"
#define SCL_H   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
#define SCL_L   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
#define SDA_H   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET)
#define SDA_L   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET)
#define SDA_READ HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)

#define AS5600_ADDR 0x36 << 1  // AS5600 器件地址

//#define USE_MT6701
#define USE_AS5600
extern uint16_t MT6701_Get_Raw_Angle(void);
#if defined (USE_AS5600)
extern uint16_t _ME_AS5600_Read_Raw_Angle(void);
#endif
#endif