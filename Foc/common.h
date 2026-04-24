#ifndef __COMMON_H
#define __COMMON_H
#define System_Delay  HAL_Delay
#define PORT_T GPIO_TypeDef*
#define    u8   uint8_t
#define   u16    uint16_t
#define   u32    uint32_t

/* gate drive option */
#define USE_DRV8301
//#define USE_DRV8323
//#define USE_FD6288Q

/* led option */
//#define USE_LED

/* encoder option */
#define USE_MT6701
//#define USE_AS5600
//#define USE_HALL


#endif