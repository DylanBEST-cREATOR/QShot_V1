#ifndef __HOST_COMM_H
#define __HOST_COMM_H

#include <stdint.h>
#include "FOC_Core.h"

/* 波形显示通道枚举 */
typedef enum {
    WAVE_PHASE_I = 1,
    WAVE_CLARKE  = 2,
    WAVE_PARK    = 3,
    WAVE_SVPWM   = 4,
    WAVE_ANGLE   = 5
} HostComm_WaveMode_e;

/* 控制模式枚举 */
typedef enum {
    MODE_IDLE = 0,
    MODE_CURR = 1,
    MODE_SPD  = 2
} HostComm_CtrlMode_e;

/* 单点快照 */
typedef struct {
    uint32_t seq;
    int32_t  ch1;
    int32_t  ch2;
    int32_t  ch3;
    uint16_t vbus;   // ADC raw
    int32_t  speed;  // rpm
    int32_t  iq;     // q16
} HostComm_Snapshot_t;

/* 队列深度 */
#define HOSTCOMM_TXQ_DEPTH      256u

/* ===== V1.5-P03 上行二进制帧 =====
 * SOF1 SOF2 LEN_L LEN_H TYPE PAYLOAD CRC_L CRC_H
 * SOF  = A5 5A
 * TYPE = 0x01
 * payload = seq(4)+ch1(4)+ch2(4)+ch3(4)+vbus(2)+speed(4)+iq(4)=26
 * frame = 2+2+1+26+2 = 33
 */
#define HOSTCOMM_SOF1         0xA5u
#define HOSTCOMM_SOF2         0x5Au
#define HOSTCOMM_TYPE_DATA    0x01u
#define HOSTCOMM_PAYLOAD_LEN  26u
#define HOSTCOMM_FRAME_LEN    33u
#define HOSTCOMM_MAX_FRAME    HOSTCOMM_FRAME_LEN

typedef struct {
    FOC_Core_t *foc;
    int32_t    *motor_speed_rpm;

    HostComm_WaveMode_e wave_mode;
    HostComm_CtrlMode_e ctrl_mode;
    int32_t             setpoint;

    uint16_t downsample_rate;
    uint16_t ds_cnt;
    uint32_t sample_seq;

    int64_t acc_ch1, acc_ch2, acc_ch3;
    int64_t acc_vbus, acc_speed, acc_iq;

    volatile uint16_t q_head;
    volatile uint16_t q_tail;
    volatile uint16_t q_count;
    HostComm_Snapshot_t txq[HOSTCOMM_TXQ_DEPTH];
} HostComm_Manager_t;

void HostComm_Init(HostComm_Manager_t *manager, FOC_Core_t *foc_core, int32_t *speed_ptr);
void HostComm_ISR_Snapshot(HostComm_Manager_t *manager);
void HostComm_Main_Process(HostComm_Manager_t *manager);
void HostComm_Rx_Parse(HostComm_Manager_t *manager, uint8_t *buf, uint32_t len);

#endif
