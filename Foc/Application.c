#include "Application.h"


u16 Response1 = 0;
uint16_t drv8233_rt(uint8_t ReadWrite, uint8_t Address, uint16_t Data);
System_Status_t System_Status={0};
void MySystemInit(void){
	#if defined(USE_LED)
	LED_Init();
	#endif
	#if defined(USE_DRV8323)
	DRV8323_Init();
	#elif defined(USE_DRV8301)
	DRV8301_Init();
	#else
	FD6288Q_Init();
	#endif
	
	}

	#if defined(USE_LED)
void LEDStatus_Function(System_Status_t *System_Status){
	if(System_Status->Voltage_Error == 1){
//		System_Status->System_OK = 0;
		LED_Error();
	}
	else if(System_Status->Current_Error ==1){
//		System_Status->System_OK = 0;
		LED_Error();
	}
  else if(System_Status->Speed_Error == 1){
//		System_Status->System_OK = 0;
		LED_Error();
	}
	else if(System_Status->Position_Error == 1){
//		System_Status->System_OK = 0;
		LED_Error();
	}
	else if(System_Status->Else_Error == 1){
//		System_Status->System_OK = 0;
		LED_Error();
	}
	else
	System_Status->System_OK = 1;
	LED_Run();
  
}
#endif

//void GateDriverInit(void){
//	DRV8323_Enable();
//	DRV8323_CAL();
//	System_Delay(10);

//	drv8233_rt(0, 0X02,0x20);
//	drv8233_rt(0, 0X03,0x3AF);
//	drv8233_rt(0, 0X04,0x7AF);
//	
//	drv8233_rt(1, 0X02,0x00);
//	drv8233_rt(1, 0X03,0x00);
//	drv8233_rt(1, 0X04,0x00);





//	
//}




//uint16_t drv8233_rt(uint8_t ReadWrite, uint8_t Address, uint16_t Data)
//{
//    uint16_t command = 0;
//    uint16_t response2 = 0;

//    // 构造16位命令
//    command |= (ReadWrite << 15);
//    command |= ((Address & 0x0F) << 11);
//    if (!ReadWrite) {
//        command |= (Data & 0x07FF);
//    }
//    TxArray[0] = command;
////	  OLED_ShowHexNum(1, 1, command, 4);
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
////    response = SPI3_ReadWriteByte(command); // 发送获取数据
////		Response1 = HAL_SPI_TransmitReceive(&hspi2,(uint8_t*)TxArray,(uint8_t*)RxArray,1,500);
//		HAL_SPI_Transmit(&hspi2,(uint8_t*)TxArray,1,500);
////    DRV8323_CS= 1; // 取消片选
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);

////		OLED_ShowHexNum(2, 1, response, 4);

//    return response2;
//}