#include "Midware.h"


void DRV8323Spi_CsReSet(void);
void DRV8323Spi_CsSet(void);
u16 DriverSpi_SendReceive(u16* Txdata,u16* Rxdata,u16 Size,u32 Timeout);
u16 EncoderSpi_SendReceive(u8* Txdata, u8* Rxdata, u16 Size, u32 Timeout);
void Reset_Pin(PORT_T PORT ,u16 PIN);
void Set_Pin(PORT_T PORT ,u16 PIN);

/* jscope static value*/
u8 DRV8301_ID;


/* led midware */
#if defined(USE_LED)
void TogglePin_500ms(PORT_T PORT ,u16 PIN){
	HAL_GPIO_TogglePin(PORT,PIN);
	System_Delay(500);
}
	


void LED_Run(void){
	TogglePin_500ms(RUN_PORT,RUN_PIN);
	
}
void LED_Error(void){
	Reset_Pin(ERROR_PORT,ERROR_PIN);
}

void LED_Init(void){
//	Reset_TwoPin();
	Reset_Pin(RUN_PORT,RUN_PIN);
	
}
#endif
/* drv8323 midware */

#if defined(USE_DRV8323)
void DRV8323_Enable(void){
	Set_Pin(DRV8323_ENABLE_PORT,DRV8323_ENABLE_PIN);
}


void DRV8323_Disable(void){
	Reset_Pin(DRV8323_ENABLE_PORT,DRV8323_ENABLE_PIN);
}

void DRV8323_CAL(void){
	Set_Pin(CAL_PORT,CAL_PIN);
	System_Delay(2);
	Reset_Pin(CAL_PORT,CAL_PIN);
}

void DRV8323Spi_CsReSet(void){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
  }

void DRV8323Spi_CsSet(void){
HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
}



u16 DRV8323_Readreg(const DRV8323_Reg reg){
    uint16_t controlword = 0x8000 | reg; // MSbit =1 for read, address is 3 bits (MSbit is always 0), data is 11 bits
    uint16_t recbuff = 0xbeef;
    // CS
	  DRV8323Spi_CsSet();
    DRV8323Spi_CsReSet();
	  uint16_t Txdata = controlword;
	  uint16_t Rxdata = recbuff;
//    recbuff = SPI_RW_byte(1, controlword);
	  Rxdata = DriverSpi_SendReceive(&Txdata,&Rxdata,1,500);
    // NCS
   DRV8323Spi_CsSet();
    return (0x7ff & Rxdata);
}

void DRV8323_Writereg(const DRV8323_Reg reg, uint16_t regVal){
    uint16_t controlword = (reg | (regVal & 0x7ff)); // MSbit =0 for write, address is 3 bits (MSbit is always 0), data is 11 bits

	  DRV8323Spi_CsSet();
    DRV8323Spi_CsReSet();
	  uint16_t Txdata = controlword;
	  uint16_t Rxdata = 0;
	 
	  DriverSpi_SendReceive(&Txdata,&Rxdata,1,500);

    DRV8323Spi_CsSet();
}

u16  DRV8323_Read_FSR1(void){
     DRV8323_Readreg(DRV8323_FAULT_STATUS_1);
}

u16 DRV8323_Read_FSR2(void){
    return DRV8323_Readreg(DRV8323_FAULT_STATUS_2);
}

void DRV8323_Write_DCR(int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT){
    uint16_t val = (DIS_CPUV << 9) | (DIS_GDF << 8) | (OTW_REP << 7) | (PWM_MODE << 5) | (PWM_COM << 4) | (PWM_DIR << 3) | (COAST << 2) | (BRAKE << 1) | CLR_FLT;
    DRV8323_Writereg(DRV8323_DRIVER_CONTROL, val);
}

void DRV8323_Write_HSR(int LOCK, int IDRIVEP_HS, int IDRIVEN_HS){
    uint16_t val = (LOCK << 8) | (IDRIVEP_HS << 4) | IDRIVEN_HS;
    DRV8323_Writereg(DRV8323_GATE_DRIVE_HS, val);
}

void DRV8323_Write_LSR(int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS){
    uint16_t val = (CBC << 10) | (TDRIVE << 8) | (IDRIVEP_LS << 4) | IDRIVEN_LS;
    DRV8323_Writereg(DRV8323_GATE_DRIVE_LS, val);
}

void DRV8323_Write_OCPCR(int TRETRY, int DEAD_TIME, int OCP_MODE, int OCP_DEG, int VDS_LVL){
    uint16_t val = (TRETRY << 10) | (DEAD_TIME << 8) | (OCP_MODE << 6) | (OCP_DEG << 4) | VDS_LVL;
    DRV8323_Writereg(DRV8323_OCP_CONTROL, val);
}

void DRV8323_Write_CSACR(int CSA_FET, int VREF_DIV, int LS_REF, int CSA_GAIN, int DIS_SEN, int CSA_CAL_A, int CSA_CAL_B, int CSA_CAL_C, int SEN_LVL){
    uint16_t val = (CSA_FET << 10) | (VREF_DIV << 9) | (LS_REF << 8) | (CSA_GAIN << 6) | (DIS_SEN << 5) | (CSA_CAL_A << 4) | (CSA_CAL_B << 3) | (CSA_CAL_C << 2) | SEN_LVL;
    DRV8323_Writereg(DRV8323_CSA_CONTROL, val);
}


void DRV8323_Init(void){
	  DRV8323_Disable();
	  System_Delay(1);
		DRV8323_Enable();
    DRV8323_CAL();
	  DRV8323_Write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
    System_Delay(1);
    DRV8323_Write_HSR(LOCK_OFF, IDRIVEP_HS_370MA, IDRIVEN_HS_1360MA);
    System_Delay(1);
    DRV8323_Write_LSR(1, TDRIVE_1000NS, IDRIVEP_LS_370MA, IDRIVEN_LS_1360MA);
    System_Delay(1);
    DRV8323_Write_CSACR(0x0, VREF_DIV_2, 0x0, CSA_GAIN_20, 0x1, 0x0, 0x0, 0x0, SEN_LVL_0_25);
    System_Delay(1);
    DRV8323_Write_OCPCR(TRETRY_50US, DEADTIME_50NS, OCP_NONE, OCP_DEG_4US, VDS_LVL_0_45);
    System_Delay(1);

}
#endif

/* drv8301 midware */
#if defined(USE_DRV8301)
void DRV8301_Enable(void){
    Set_Pin(DRV8301_ENABLE_PORT,DRV8301_ENABLE_PIN);
}

void DRV8301_Disable(void){
		Reset_Pin(DRV8301_ENABLE_PORT,DRV8301_ENABLE_PIN);
}
void DRV8301Spi_CsSet(void){
		Set_Pin(DRV8301_CS_PORT,DRV8301_CS_PIN);

}

void DRV8301Spi_CsReSet(void){
		Reset_Pin(DRV8301_CS_PORT,DRV8301_CS_PIN);
}


/* drv8301 midware */


/**
 * @brief DRV8301 写寄存器 (8位 SPI 兼容版)
 */
u16 DRV8301_Writereg(const DRV8301_Reg reg, uint16_t regVal) {
    uint16_t controlword = (reg | (regVal & 0x7ff)); 
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    // --- 关键：手动拆分为大端序 ---
    tx_buf[0] = (uint8_t)(controlword >> 8);   // 第1个字节发高8位 (包含地址和R/W)
    tx_buf[1] = (uint8_t)(controlword & 0xFF); // 第2个字节发低8位

    DRV8301Spi_CsReSet(); // 拉低 CS
    
    // 调用底层发送。注意：在8位模式下，Size 必须是 2 (表示2个字节)
    // 这里强制转换指针类型，确保 HAL 库按字节顺序读取 tx_buf
    DriverSpi_SendReceive((u16*)tx_buf, (u16*)rx_buf, 2, 500);
    
    DRV8301Spi_CsSet();   // 拉高 CS

    // 组合接收到的数据 (同样需要手动处理字节序)
    return ((u16)rx_buf[0] << 8) | rx_buf[1];
}

/**
 * @brief DRV8301 读寄存器 (8位 SPI 兼容版)
 * DRV8301 协议：第一帧发读指令，第二帧才返回数据
 */
u16 DRV8301_Readreg_Twice(const DRV8301_Reg reg) {
    // 构造读指令：MSB = 1
    uint16_t controlword = 0x8000 | reg; 
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = (uint8_t)(controlword >> 8);
    tx_buf[1] = (uint8_t)(controlword & 0xFF);

    // 第一帧：发送读指令，忽略返回值
    DRV8301Spi_CsReSet();
    DriverSpi_SendReceive((u16*)tx_buf, (u16*)rx_buf, 2, 500);
    DRV8301Spi_CsSet();

    // 适当延时，确保芯片准备好数据（通常 SPI 间隔即可，若失败可加 1us 延时）
    
    // 第二帧：发送 Dummy 指令（通常读 Status1），获取上一帧请求的数据
    // 这里直接复用 Writereg 逻辑发一个空指令
    u16 raw_data = DRV8301_Writereg(DRV8301_STATUS_1, 0x0000);
    
    return (raw_data & 0x07FF); // 返回低11位有效数据
}






u16  DRV8301_ReadStatus_Register1(void){
     return DRV8301_Readreg_Twice( DRV8301_STATUS_1);
}

u16  DRV8301_ReadStatus_Register2(void){
     return DRV8301_Readreg_Twice( DRV8301_STATUS_2);
}

u16 DRV8301_Read_DeviceID(void){
    u16 ID =  DRV8301_ReadStatus_Register2();
		return (u16)(ID&0x000f);

}


void DRV8301_WriteControl_Register1(uint8_t Gate_Cur,uint8_t Gate_RST,uint8_t PWM_Mode,uint8_t OCP_Mode,uint8_t OC_ADJ){
		uint16_t val = (Gate_Cur) | (Gate_RST << 2) | (PWM_Mode << 3) | (OCP_Mode << 4) | (OC_ADJ << 6);
	  DRV8301_Writereg(DRV8301_CONTROL_1,val);
}

void DRV8301_WriteControl_Register2(u8 OCTW_Mode,u8 GAIN,u8 DC_CAL_CH1,u8 DC_CAL_CH2,u8 OC_TOFF){
		uint16_t val = (OCTW_Mode) | (GAIN << 2) | (DC_CAL_CH1 << 4) | (DC_CAL_CH2 << 5) | (OC_TOFF << 6);
	  DRV8301_Writereg(DRV8301_CONTROL_2,val);
}

/**
 * @brief  DRV8301 详细故障诊断与过流解析
 * @return 0: 无故障, 1: 存在故障
 */
uint8_t DRV8301_Diagnose(void) {
    // 1. 读取状态寄存器并提取低 11 位有效数据
    // 注意：Readreg_Twice 内部已包含 dummy read 逻辑
    uint16_t sr1 = DRV8301_ReadStatus_Register1() & 0x07FF; 
    uint16_t sr2 = DRV8301_ReadStatus_Register2() & 0x07FF;

    // 2. 检查是否有总故障标志 (D10: FAULT)
    if (sr1 & DRV8301_STAT1_FAULT) {
        
        // --- A. 系统级严重故障判定 ---
        if (sr1 & DRV8301_STAT1_OTSD)    { /* 报错: 芯片过温关断 */ }
        if (sr1 & DRV8301_STAT1_PVDD_UV) { /* 报错: PVDD 欠压 */ }
        if (sr1 & DRV8301_STAT1_GVDD_UV) { /* 报错: GVDD 欠压 */ }

        // --- B. 详细过流故障解析 (D5-D0) ---
        // 掩码 0x3F 检查是否有任何一相发生过流
        if (sr1 & 0x003F) {
            
            // --- A 相过流检查 ---
            if (sr1 & DRV8301_STAT1_FETHA_OC) {
                // A相 高侧(High-side)过流。可能原因：相线对地短路或下管击穿
            }
            if (sr1 & DRV8301_STAT1_FETLA_OC) {
                // A相 低侧(Low-side)过流。可能原因：相线对电源短路或上管击穿
            }

            // --- B 相过流检查 ---
            if (sr1 & DRV8301_STAT1_FETHB_OC) {
                // B相 高侧过流
            }
            if (sr1 & DRV8301_STAT1_FETLB_OC) {
                // B相 低侧过流
            }

            // --- C 相过流检查 ---
            if (sr1 & DRV8301_STAT1_FETHC_OC) {
                // C相 高侧过流
            }
            if (sr1 & DRV8301_STAT1_FETLC_OC) {
                // C相 低侧过流
            }
            
            /* 建议在这里执行停机逻辑: 例如强制 PWM 占空比为 0 或封锁驱动 */
        }
        
        return 1; // 返回 1 表示检测到严重故障
    }

    // 3. 检查非紧急警告 (不触发 nFAULT 引脚，但需要关注)
    if (sr1 & DRV8301_STAT1_OTW) {
        // 警告: 芯片温度过高，建议降低电流运行
    }

    if (sr2 & DRV8301_STAT2_GVDD_OV) {
        // 警告: GVDD 电压过高
        return 1;
    }

    return 0; // 无故障，一切正常
}





void DRV8301_Init(void){
	DRV8301_Disable();
	System_Delay(1);
	DRV8301_Enable();
	System_Delay(20);  //Important   //drv8323 failed reason
	DRV8301_WriteControl_Register1(CONTROL1_GCU_0_25A,CONTROL1_GCR_NORMAL,CONTROL1_GPM_6,CONTROL1_OCPM_CL,CONTROL1_OCADJ_0_197V);
	DRV8301_WriteControl_Register2(CONTROL2_OCTW_BOTH,CONTROL2_GAIN_20,CONTROL2_CAL_CH1,CONTROL2_CAL_CH2,CONTROL2_OCTOFF_CBC);
	DRV8301_ID = DRV8301_Read_DeviceID();
	if(DRV8301_Diagnose() == 0){ 
	}
//	else{
//////		DRV8301_Disable();
//	}
//	
	
	

}

#endif

#if defined(USE_FD6288Q)

void FD6288Q_Init(void){

}
#endif

#if defined(USE_MT6701)

void MT6701Spi_CsSet(void){
		Set_Pin(MT6701_CS_PORT,MT6701_CS_PIN);
}
void MT6701Spi_CsReset(void){
		Reset_Pin(MT6701_CS_PORT,MT6701_CS_PIN);
}

float MT6701_GetAngle(void){
    
	  uint8_t tx_buf[3] = {0};
    uint8_t rx_buf[3] = {0};
		MT6701Spi_CsSet();
		MT6701Spi_CsReset();
	  EncoderSpi_SendReceive(tx_buf, rx_buf, 3, 100);
		MT6701Spi_CsSet();
		uint32_t data = (rx_buf[0]<<16)|(rx_buf[1]<<8) | rx_buf[2];
		uint16_t angle = (data>>10);
		return ((float)angle*360)/MT6701_CPR;
}
#endif


void Reset_Pin(PORT_T PORT ,u16 PIN){
	HAL_GPIO_WritePin(PORT,PIN,GPIO_PIN_RESET);

}

void Set_Pin(PORT_T PORT ,u16 PIN){
	HAL_GPIO_WritePin(PORT,PIN,GPIO_PIN_SET);
}

u16 DriverSpi_SendReceive(u16* Txdata, u16* Rxdata, u16 Size, u32 Timeout){
    HAL_StatusTypeDef status;
    status = HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)Txdata, (uint8_t*)Rxdata, Size, Timeout);
    
    if(status == HAL_OK) {
        return *Rxdata; // 必须返回解引用后的数据
    }
    return 0;
}

u16 EncoderSpi_SendReceive(u8* Txdata, u8* Rxdata, u16 Size, u32 Timeout){
   HAL_StatusTypeDef status;
   status = HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)Txdata, (uint8_t*)Rxdata, Size, Timeout);
    
   if(status == HAL_OK) {
        return *Rxdata; // 必须返回解引用后的数据
    }
   return 0;

}




