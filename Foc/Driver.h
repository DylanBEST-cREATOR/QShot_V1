#ifndef __DRIVER_H
#define __DRIVER_H
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "common.h"
/* LED DRIVER */
#define RUN_PORT GPIOB
#define RUN_PIN  GPIO_PIN_4
#define ERROR_PORT GPIOB
#define ERROR_PIN  GPIO_PIN_3

extern void TogglePin_500ms(GPIO_TypeDef* PORT ,u16 PIN);
extern void Reset_Pin(GPIO_TypeDef* PORT ,u16 PIN);
extern void Set_Pin(GPIO_TypeDef* PORT ,u16 PIN);




#endif
