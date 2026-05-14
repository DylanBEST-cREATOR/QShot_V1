#ifndef __MY_GATEDRIVER_H
#define __MY_GATEDRIVER_H
#include "stm32f4xx_hal.h"
#include "spi.h"
#define USE_DRV8301



#if defined(USE_DRV8323)
/* DRV8323 midware */
#define DRV8323_ENABLE_PORT GPIOB
#define DRV8323_ENABLE_PIN  GPIO_PIN_11
#define DRV8323_CS_PORT 
#define CAL_PORT    GPIOB
#define CAL_PIN     GPIO_PIN_10
#define FAULT_ADDR     0X00
#define VGS_ADDR       0X01
#define CONTROL_ADDR   0X02
#define HS_ADDR        0X03
#define LS_ADDR        0X04
#define OCP_ADDR       0X05
#define CSA_ADDR       0X06





typedef enum
{
    DRV8323_FAULT_STATUS_1 = 0 << 11,
    DRV8323_FAULT_STATUS_2 = 1 << 11,
    DRV8323_DRIVER_CONTROL = 2 << 11,
    DRV8323_GATE_DRIVE_HS = 3 << 11,
    DRV8323_GATE_DRIVE_LS = 4 << 11,
    DRV8323_OCP_CONTROL = 5 << 11,
    DRV8323_CSA_CONTROL = 6 << 11,
} DRV8323_Reg;

/// Drive Control Fields ///
#define DIS_CPUV_EN 0x0 /// Charge pump UVLO fault
#define DIS_CPUV_DIS 0x1
#define DIS_GDF_EN 0x0 /// Gate drive fauilt
#define DIS_GDF_DIS 0x1
#define OTW_REP_EN 0x1 /// Over temp warning reported on nFAULT/FAULT bit
#define OTW_REP_DIS 0x0
#define PWM_MODE_6X 0x0 /// PWM Input Modes
#define PWM_MODE_3X 0x1
#define PWM_MODE_1X 0x2
#define PWM_MODE_IND 0x3
#define PWM_1X_COM_SYNC 0x0 /// 1x PWM Mode synchronou rectification
#define PWM_1X_COM_ASYNC 0x1
#define PWM_1X_DIR_0 0x0 /// In 1x PWM mode this bit is ORed with the INHC (DIR) input
#define PWM_1X_DIR_1 0x1

/// Gate Drive HS Fields ///
#define LOCK_ON 0x6
#define LOCK_OFF 0x3
#define IDRIVEP_HS_10MA 0x0 /// Gate drive high side turn on current
#define IDRIVEP_HS_30MA 0x1
#define IDRIVEP_HS_60MA 0x2
#define IDRIVEP_HS_80MA 0x3
#define IDRIVEP_HS_120MA 0x4
#define IDRIVEP_HS_140MA 0x5
#define IDRIVEP_HS_170MA 0x6
#define IDRIVEP_HS_190MA 0x7
#define IDRIVEP_HS_260MA 0x8
#define IDRIVEP_HS_330MA 0x9
#define IDRIVEP_HS_370MA 0xA
#define IDRIVEP_HS_440MA 0xB
#define IDRIVEP_HS_570MA 0xC
#define IDRIVEP_HS_680MA 0xD
#define IDRIVEP_HS_820MA 0xE
#define IDRIVEP_HS_1000MA 0xF
#define IDRIVEN_HS_20MA 0x0 /// High side turn off current
#define IDRIVEN_HS_60MA 0x1
#define IDRIVEN_HS_120MA 0x2
#define IDRIVEN_HS_160MA 0x3
#define IDRIVEN_HS_240MA 0x4
#define IDRIVEN_HS_280MA 0x5
#define IDRIVEN_HS_340MA 0x6
#define IDRIVEN_HS_380MA 0x7
#define IDRIVEN_HS_520MA 0x8
#define IDRIVEN_HS_660MA 0x9
#define IDRIVEN_HS_740MA 0xA
#define IDRIVEN_HS_880MA 0xB
#define IDRIVEN_HS_1140MA 0xC
#define IDRIVEN_HS_1360MA 0xD
#define IDRIVEN_HS_1640MA 0xE
#define IDRIVEN_HS_2000MA 0xF

/// Gate Drive LS Fields ///
#define TDRIVE_500NS 0x0 /// Peak gate-current drive time
#define TDRIVE_1000NS 0x1
#define TDRIVE_2000NS 0x2
#define TDRIVE_4000NS 0x3
#define IDRIVEP_LS_10MA 0x0 /// Gate drive high side turn on current
#define IDRIVEP_LS_30MA 0x1
#define IDRIVEP_LS_60MA 0x2
#define IDRIVEP_LS_80MA 0x3
#define IDRIVEP_LS_120MA 0x4
#define IDRIVEP_LS_140MA 0x5
#define IDRIVEP_LS_170MA 0x6
#define IDRIVEP_LS_190MA 0x7
#define IDRIVEP_LS_260MA 0x8
#define IDRIVEP_LS_330MA 0x9
#define IDRIVEP_LS_370MA 0xA
#define IDRIVEP_LS_440MA 0xB
#define IDRIVEP_LS_570MA 0xC
#define IDRIVEP_LS_680MA 0xD
#define IDRIVEP_LS_820MA 0xE
#define IDRIVEP_LS_1000MA 0xF
#define IDRIVEN_LS_20MA 0x0 /// High side turn off current
#define IDRIVEN_LS_60MA 0x1
#define IDRIVEN_LS_120MA 0x2
#define IDRIVEN_LS_160MA 0x3
#define IDRIVEN_LS_240MA 0x4
#define IDRIVEN_LS_280MA 0x5
#define IDRIVEN_LS_340MA 0x6
#define IDRIVEN_LS_380MA 0x7
#define IDRIVEN_LS_520MA 0x8
#define IDRIVEN_LS_660MA 0x9
#define IDRIVEN_LS_740MA 0xA
#define IDRIVEN_LS_880MA 0xB
#define IDRIVEN_LS_1140MA 0xC
#define IDRIVEN_LS_1360MA 0xD
#define IDRIVEN_LS_1640MA 0xE
#define IDRIVEN_LS_2000MA 0xF

/// OCP Control Fields ///
#define TRETRY_4MS 0x0 /// VDS OCP and SEN OCP retry time
#define TRETRY_50US 0x1
#define DEADTIME_50NS 0x0 /// Deadtime
#define DEADTIME_100NS 0x1
#define DEADTIME_200NS 0x2
#define DEADTIME_400NS 0x3
#define OCP_LATCH 0x0 /// OCP Mode
#define OCP_RETRY 0x1
#define OCP_REPORT 0x2
#define OCP_NONE 0x3
#define OCP_DEG_2US 0x0 /// OCP Deglitch Time
#define OCP_DEG_4US 0x1
#define OCP_DEG_6US 0x2
#define OCP_DEG_8US 0x3
#define VDS_LVL_0_06 0x0
#define VDS_LVL_0_13 0x1
#define VDS_LVL_0_2 0x2
#define VDS_LVL_0_26 0x3
#define VDS_LVL_0_31 0x4
#define VDS_LVL_0_45 0x5
#define VDS_LVL_0_53 0x6
#define VDS_LVL_0_6 0x7
#define VDS_LVL_0_68 0x8
#define VDS_LVL_0_75 0x9
#define VDS_LVL_0_94 0xA
#define VDS_LVL_1_13 0xB
#define VDS_LVL_1_3 0xC
#define VDS_LVL_1_5 0xD
#define VDS_LVL_1_7 0xE
#define VDS_LVL_1_88 0xF

/// CSA Control Fields ///
#define CSA_FET_SP 0x0 /// Current sense amplifier positive input
#define CSA_FET_SH 0x1
#define VREF_DIV_1 0x0 /// Amplifier reference voltage is VREV/1
#define VREF_DIV_2 0x1 /// Amplifier reference voltage is VREV/2
#define CSA_GAIN_5 0x0 /// Current sensor gain
#define CSA_GAIN_10 0x1
#define CSA_GAIN_20 0x2
#define CSA_GAIN_40 0x3
#define DIS_SEN_EN 0x0 /// Overcurrent Fault
#define DIS_SEN_DIS 0x1
#define SEN_LVL_0_25 0x0 /// Sense OCP voltage level
#define SEN_LVL_0_5 0x1
#define SEN_LVL_0_75 0x2
#define SEN_LVL_1_0 0x3


//extern u16 DRV8323_Readreg(const DRV8323_Reg reg);
//extern void DRV8323_Writereg(const DRV8323_Reg reg, uint16_t regVal);
//extern u16  DRV8323_read_FSR1(void);
//extern u16 DRV8323_read_FSR2(void);
//extern void DRV8323_write_DCR(int DIS_CPUV, int DIS_GDF, int OTW_REP, int PWM_MODE, int PWM_COM, int PWM_DIR, int COAST, int BRAKE, int CLR_FLT);
//extern void DRV8323_write_HSR(int LOCK, int IDRIVEP_HS, int IDRIVEN_HS);
//extern void DRV8323_write_LSR(int CBC, int TDRIVE, int IDRIVEP_LS, int IDRIVEN_LS);
extern void DRV8323_Init(void);
extern void DRV8323_Enable(void);
extern void DRV8323_Disable(void);
extern void DRV8323_CAL(void);
#endif

/* drv8301 midware */

#if defined(USE_DRV8301)

#define DRV8301_ENABLE_PORT    GPIOB
#define DRV8301_ENABLE_PIN     GPIO_PIN_12

#define DRV8301_CS_PORT        GPIOC
#define DRV8301_CS_PIN         GPIO_PIN_13



typedef enum
{
    DRV8301_STATUS_1 = 0 << 11,
    DRV8301_STATUS_2 = 1 << 11,
    DRV8301_CONTROL_1 = 2 << 11,
    DRV8301_CONTROL_2 = 3 << 11,
    
} DRV8301_Reg;

typedef enum {
    DRV8301_STAT1_NONE       = 0x0000,
    DRV8301_STAT1_FETLC_OC   = (1 << 0),  // C相低侧过流
    DRV8301_STAT1_FETHC_OC   = (1 << 1),  // C相高侧过流
    DRV8301_STAT1_FETLB_OC   = (1 << 2),  // B相低侧过流
    DRV8301_STAT1_FETHB_OC   = (1 << 3),  // B相高侧过流
    DRV8301_STAT1_FETLA_OC   = (1 << 4),  // A相低侧过流
    DRV8301_STAT1_FETHA_OC   = (1 << 5),  // A相高侧过流
    DRV8301_STAT1_OTW        = (1 << 6),  // 过温警告
    DRV8301_STAT1_OTSD       = (1 << 7),  // 过温关断
    DRV8301_STAT1_PVDD_UV    = (1 << 8),  // PVDD 欠压 (主电源)
    DRV8301_STAT1_GVDD_UV    = (1 << 9),  // GVDD 欠压 (门极驱动电源)
    DRV8301_STAT1_FAULT      = (1 << 10)  // 总故障指示位
} DRV8301_Stat1_Code;


typedef enum {
    DRV8301_STAT2_NONE       = 0x0000,
    DRV8301_STAT2_ID_MASK    = 0x000F,    // Device ID 占低 4 位 (D3-D0)
    DRV8301_STAT2_GVDD_OV    = (1 << 7)   // GVDD 过压 (D7位) —— 现在是左移 7 位了！
} DRV8301_Stat2_Code;


/*driver control_1 field*/
#define CONTROL1_GCU_1_7A          0x0
#define CONTROL1_GCU_0_7A          0x1
#define CONTROL1_GCU_0_25A         0x2

#define CONTROL1_GCR_NORMAL        0x0
#define CONTROL1_GCR_RST           0x1

#define CONTROL1_GPM_6             0x0
#define CONTROL1_GPM_3             0x1

#define CONTROL1_OCPM_CL           0x0
#define CONTROL1_OCPM_OLSD         0x1
#define CONTROL1_OCPM_RO           0x2
#define CONTROL1_OCPM_OCD          0x31


#define CONTROL1_OCADJ_0_060V      0x00
#define CONTROL1_OCADJ_0_068V      0x01
#define CONTROL1_OCADJ_0_076V      0x02
#define CONTROL1_OCADJ_0_086V      0x03
#define CONTROL1_OCADJ_0_097V      0x04
#define CONTROL1_OCADJ_0_109V      0x05  // 推荐初始测试位 (适合并联MOS)(80A)
#define CONTROL1_OCADJ_0_123V      0x06
#define CONTROL1_OCADJ_0_138V      0x07
#define CONTROL1_OCADJ_0_155V      0x08
#define CONTROL1_OCADJ_0_175V      0x09
#define CONTROL1_OCADJ_0_197V      0x0A  // 推荐稳定运行位(80A)
#define CONTROL1_OCADJ_0_222V      0x0B
#define CONTROL1_OCADJ_0_250V      0x0C
#define CONTROL1_OCADJ_0_282V      0x0D
#define CONTROL1_OCADJ_0_317V      0x0E
#define CONTROL1_OCADJ_0_358V      0x0F
#define CONTROL1_OCADJ_0_403V      0x10
#define CONTROL1_OCADJ_0_454V      0x11
#define CONTROL1_OCADJ_0_511V      0x12
#define CONTROL1_OCADJ_0_576V      0x13
#define CONTROL1_OCADJ_0_648V      0x14
#define CONTROL1_OCADJ_0_730V      0x15
#define CONTROL1_OCADJ_0_822V      0x16
#define CONTROL1_OCADJ_0_926V      0x17
#define CONTROL1_OCADJ_1_043V      0x18
#define CONTROL1_OCADJ_1_175V      0x19
#define CONTROL1_OCADJ_1_324V      0x1A
#define CONTROL1_OCADJ_1_491V      0x1B
#define CONTROL1_OCADJ_1_679V      0x1C
#define CONTROL1_OCADJ_1_892V      0x1D
#define CONTROL1_OCADJ_2_131V      0x1E
#define CONTROL1_OCADJ_2_400V      0x1F


/* driver control_2 field (Shunt Amp & Misc) */


#define CONTROL2_OCTW_BOTH         0x0  // 
#define CONTROL2_OCTW_OT           0x1  // 
#define CONTROL2_OCTW_OC           0x2  // 


#define CONTROL2_GAIN_10           0x0  // 
#define CONTROL2_GAIN_20           0x1  // 
#define CONTROL2_GAIN_40           0x2  // 40 V/V (针对 0.5mR, 80A 推荐)
#define CONTROL2_GAIN_80           0x3  // 


#define CONTROL2_CAL1_NORMAL       0x0  // 
#define CONTROL2_CAL_CH1          0x1  // 
#define CONTROL2_CAL2_NORMAL       0x0  // 
#define CONTROL2_CAL_CH2          0x1  // 


#define CONTROL2_OCTOFF_CBC        0x0  // Cycle by Cycle (逐周期限流)
#define CONTROL2_OCTOFF_OFFTIME    0x1  // 关断时间控制






extern void DRV8301_Init(void);
extern void DRV8301_CAL(void);
extern uint8_t DRV8301_Diagnose(void);
#endif

#if defined(USE_FD6288Q)
extern void FD6288Q_Init(void);
#endif

/*encoder midware*/
#if defined(USE_MT6701)

#define MT6701_CPR 16383
#define MT6701_CS_PORT GPIOB
#define MT6701_CS_PIN  GPIO_PIN_2

extern float MT6701_GetAngle(void);


#endif









#if defined(USE_LED)
/* led midware */
#define RUN_PORT GPIOB
#define RUN_PIN  GPIO_PIN_4
#define ERROR_PORT GPIOB
#define ERROR_PIN  GPIO_PIN_3




extern void LED_Run(void);
extern void LED_Error(void);
extern void LED_Init(void);
#endif


#endif
