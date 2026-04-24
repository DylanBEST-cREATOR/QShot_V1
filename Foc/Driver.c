#include "Driver.h"

/* LED drive */
void TogglePin_500ms(GPIO_TypeDef* PORT ,u16 PIN){
	HAL_GPIO_TogglePin(PORT,PIN);
	System_Delay(500);
}
	
void Reset_Pin(GPIO_TypeDef* PORT ,u16 PIN){
	HAL_GPIO_WritePin(PORT,PIN,GPIO_PIN_RESET);

}

void Set_Pin(GPIO_TypeDef* PORT ,u16 PIN){
	HAL_GPIO_WritePin(PORT,PIN,GPIO_PIN_SET);
}

//void Reset_TwoPin(void){
////	HAL_GPIO_WritePin(ERROR_PORT,ERROR_PIN,GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(RUN_PORT,RUN_PIN,GPIO_PIN_RESET);
//}

/* drv8323 driver */
u16 DriverSpi_SendReceive(u8* Txdata,u8* Rxdata,u16 Size,u32 Timeout){
	
	HAL_SPI_TransmitReceive(&hspi2,Txdata,Rxdata,Size,Timeout);
}


