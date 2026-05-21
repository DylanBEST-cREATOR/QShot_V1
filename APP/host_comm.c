#include "host_comm.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ==================== 配置 ==================== */
#ifndef HOSTCOMM_ENABLE_ACK_TEXT
#define HOSTCOMM_ENABLE_ACK_TEXT         1
#endif

#ifndef HOSTCOMM_USE_DEFAULT_HARDSTOP
#define HOSTCOMM_USE_DEFAULT_HARDSTOP    1
#endif

/* 通信看门狗（默认关闭，避免“无心跳自动停机”） */
#ifndef HOSTCOMM_CMD_WATCHDOG_EN
#define HOSTCOMM_CMD_WATCHDOG_EN         0
#endif

#ifndef HOSTCOMM_CMD_TIMEOUT_MS
#define HOSTCOMM_CMD_TIMEOUT_MS          600u
#endif

/* 安全限幅 */
#ifndef HOSTCOMM_SPD_SET_MAX_RPM
#define HOSTCOMM_SPD_SET_MAX_RPM         1100
#endif

#ifndef HOSTCOMM_CURR_SET_MAX_Q16
#define HOSTCOMM_CURR_SET_MAX_Q16        32768   /* 0.5pu */
#endif

/* ==================== 发送缓冲 ==================== */
static uint8_t s_tx_frame[HOSTCOMM_MAX_FRAME];

/* ==================== ACK 缓冲 ==================== */
static volatile uint8_t s_ack_pending = 0;
static char s_ack_line[96];

/* ==================== 命令时间戳 ==================== */
static uint32_t s_last_cmd_ms = 0u;

/* ==================== 可重载钩子 ==================== */
__weak void HostComm_OnStopSoft(void) {}
__weak void HostComm_OnStopHard(void) {}

/* ==================== 小工具 ==================== */
static inline void wr_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}
static inline void wr_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}
static inline void wr_i32_le(uint8_t *p, int32_t v)
{
    wr_u32_le(p, (uint32_t)v);
}

/* CRC16-CCITT (poly 0x1021, init 0xFFFF) */
static uint16_t crc16_ccitt(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;
    uint32_t i;
    uint8_t b;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (b = 0u; b < 8u; b++) {
            if (crc & 0x8000u) crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else               crc <<= 1;
        }
    }
    return crc;
}

static void HostComm_SetAck(const char *s)
{
#if HOSTCOMM_ENABLE_ACK_TEXT
    uint32_t n;

    if (s == NULL) return;

    __disable_irq();
    n = (uint32_t)strlen(s);
    if (n > sizeof(s_ack_line) - 2u) n = (uint32_t)(sizeof(s_ack_line) - 2u);
    memcpy(s_ack_line, s, n);
    s_ack_line[n++] = '\n';
    s_ack_line[n] = '\0';
    s_ack_pending = 1u;
    __enable_irq();
#else
    (void)s;
#endif
}

/* ==================== 队列 ==================== */
static void HostComm_Q_Clear(HostComm_Manager_t *m)
{
    if (m == NULL) return;

    __disable_irq();
    m->q_head = 0u;
    m->q_tail = 0u;
    m->q_count = 0u;

    m->ds_cnt = 0u;
    m->acc_ch1 = 0;
    m->acc_ch2 = 0;
    m->acc_ch3 = 0;
    m->acc_vbus = 0;
    m->acc_speed = 0;
    m->acc_iq = 0;
    __enable_irq();
}

static void HostComm_Q_Push(HostComm_Manager_t *m, const HostComm_Snapshot_t *s)
{
    uint16_t head, tail, cnt;

    if (m == NULL || s == NULL) return;

    head = m->q_head;
    tail = m->q_tail;
    cnt  = m->q_count;

    if (cnt >= HOSTCOMM_TXQ_DEPTH) {
        tail = (uint16_t)((tail + 1u) % HOSTCOMM_TXQ_DEPTH);
        cnt--;
    }

    m->txq[head] = *s;
    head = (uint16_t)((head + 1u) % HOSTCOMM_TXQ_DEPTH);
    cnt++;

    m->q_head  = head;
    m->q_tail  = tail;
    m->q_count = cnt;
}

static uint8_t HostComm_Q_PopOne(HostComm_Manager_t *m, HostComm_Snapshot_t *out)
{
    uint8_t ok = 0u;
    uint16_t tail;

    if (m == NULL || out == NULL) return 0u;

    __disable_irq();
    if (m->q_count > 0u) {
        tail = m->q_tail;
        *out = m->txq[tail];
        m->q_tail = (uint16_t)((tail + 1u) % HOSTCOMM_TXQ_DEPTH);
        m->q_count--;
        ok = 1u;
    }
    __enable_irq();

    return ok;
}

/* ==================== 停机 ==================== */
static void HostComm_DoSoftStop(HostComm_Manager_t *m)
{
    if (m == NULL || m->foc == NULL) return;

    m->ctrl_mode = MODE_IDLE;
    m->setpoint = 0;

    m->foc->pid_d.Ref = 0;
    m->foc->pid_q.Ref = 0;
    m->foc->pid_d.Integral = 0;
    m->foc->pid_q.Integral = 0;

    HostComm_Q_Clear(m);
    HostComm_OnStopSoft();
}

static void HostComm_DoHardStop(HostComm_Manager_t *m)
{
    HostComm_DoSoftStop(m);

#if HOSTCOMM_USE_DEFAULT_HARDSTOP
    /* 你的工程一般有 htim1；若没有可把宏置0 */
    extern TIM_HandleTypeDef htim1;
    __HAL_TIM_MOE_DISABLE(&htim1);
#endif

    HostComm_OnStopHard();
}

/* ==================== 构帧 ==================== */
static uint16_t HostComm_BuildDataFrame(const HostComm_Snapshot_t *s, uint8_t *out)
{
    uint16_t idx;
    uint16_t crc;

    if (s == NULL || out == NULL) return 0u;

    idx = 0u;
    out[idx++] = HOSTCOMM_SOF1;
    out[idx++] = HOSTCOMM_SOF2;

    wr_u16_le(&out[idx], HOSTCOMM_PAYLOAD_LEN); idx += 2u;
    out[idx++] = HOSTCOMM_TYPE_DATA;

    wr_u32_le(&out[idx], s->seq);   idx += 4u;
    wr_i32_le(&out[idx], s->ch1);   idx += 4u;
    wr_i32_le(&out[idx], s->ch2);   idx += 4u;
    wr_i32_le(&out[idx], s->ch3);   idx += 4u;
    wr_u16_le(&out[idx], s->vbus);  idx += 2u;
    wr_i32_le(&out[idx], s->speed); idx += 4u;
    wr_i32_le(&out[idx], s->iq);    idx += 4u;

    crc = crc16_ccitt(&out[2], (uint32_t)(2u + 1u + HOSTCOMM_PAYLOAD_LEN));
    wr_u16_le(&out[idx], crc); idx += 2u;

    return idx; /* 33 */
}

/* ==================== 命令时间戳 ==================== */
static void HostComm_CmdTouch(void)
{
    s_last_cmd_ms = HAL_GetTick();
}

/* ==================== API ==================== */
void HostComm_Init(HostComm_Manager_t *manager, FOC_Core_t *foc_core, int32_t *speed_ptr)
{
    if (manager == NULL) return;

    memset(manager, 0, sizeof(HostComm_Manager_t));
    manager->foc = foc_core;
    manager->motor_speed_rpm = speed_ptr;
    manager->wave_mode = WAVE_PHASE_I;
    manager->ctrl_mode = MODE_IDLE;
    manager->setpoint = 0;

    /* P05: 20k/20 = 500kHz */
    manager->downsample_rate = 40u;

    HostComm_Q_Clear(manager);
    HostComm_CmdTouch();
}

void HostComm_ISR_Snapshot(HostComm_Manager_t *manager)
{
    int32_t ch1 = 0, ch2 = 0, ch3 = 0;
    int32_t speed, iq;
    uint16_t vbus;
    uint16_t n;
    HostComm_Snapshot_t s;

    if (manager == NULL || manager->foc == NULL) return;

    switch (manager->wave_mode) {
        case WAVE_PHASE_I:
            ch1 = manager->foc->current.Ia;
            ch2 = manager->foc->current.Ib;
            ch3 = manager->foc->current.Ic;
            break;
        case WAVE_CLARKE:
            ch1 = manager->foc->clarke_i.alpha;
            ch2 = manager->foc->clarke_i.beta;
            ch3 = 0;
            break;
        case WAVE_PARK:
            ch1 = manager->foc->park_i.d;
            ch2 = manager->foc->park_i.q;
            ch3 = 0;
            break;
        case WAVE_SVPWM:
            ch1 = manager->foc->svpwm.Duty_A;
            ch2 = manager->foc->svpwm.Duty_B;
            ch3 = manager->foc->svpwm.Duty_C;
            break;
        case WAVE_ANGLE:
            ch1 = manager->foc->angle;
            ch2 = 0;
            ch3 = 0;
            break;
        default:
            break;
    }

    speed = (manager->motor_speed_rpm != NULL) ? *(manager->motor_speed_rpm) : 0;
    iq    = manager->foc->park_i.q;
    vbus  = manager->foc->adc_vbus_raw;

    manager->acc_ch1 += ch1;
    manager->acc_ch2 += ch2;
    manager->acc_ch3 += ch3;
    manager->acc_speed += speed;
    manager->acc_iq += iq;
    manager->acc_vbus += vbus;

    manager->ds_cnt++;
    if (manager->ds_cnt < manager->downsample_rate) return;

    n = manager->downsample_rate;
    manager->ds_cnt = 0u;

    s.seq   = ++manager->sample_seq;
    s.ch1   = (int32_t)(manager->acc_ch1 / (int64_t)n);
    s.ch2   = (int32_t)(manager->acc_ch2 / (int64_t)n);
    s.ch3   = (int32_t)(manager->acc_ch3 / (int64_t)n);
    s.vbus  = (uint16_t)(manager->acc_vbus / (int64_t)n);
    s.speed = (int32_t)(manager->acc_speed / (int64_t)n);
    s.iq    = (int32_t)(manager->acc_iq / (int64_t)n);

    manager->acc_ch1 = 0;
    manager->acc_ch2 = 0;
    manager->acc_ch3 = 0;
    manager->acc_vbus = 0;
    manager->acc_speed = 0;
    manager->acc_iq = 0;

    HostComm_Q_Push(manager, &s);
}

void HostComm_Main_Process(HostComm_Manager_t *manager)
{
    HostComm_Snapshot_t one;
    uint16_t len;
#if HOSTCOMM_ENABLE_ACK_TEXT
    uint16_t n;
#endif

    if (manager == NULL) return;

#if HOSTCOMM_CMD_WATCHDOG_EN
    if (manager->ctrl_mode != MODE_IDLE) {
        uint32_t now = HAL_GetTick();
        if ((now - s_last_cmd_ms) > HOSTCOMM_CMD_TIMEOUT_MS) {
            HostComm_DoSoftStop(manager);
        }
    }
#endif

    if (!CDC_IsTxReady_FS()) return;

#if HOSTCOMM_ENABLE_ACK_TEXT
    if (s_ack_pending) {
        __disable_irq();
        s_ack_pending = 0u;
        __enable_irq();

        n = (uint16_t)strlen(s_ack_line);
        if (n > 0u) {
            (void)CDC_Transmit_FS((uint8_t*)s_ack_line, n);
            return;
        }
    }
#endif

    if (!HostComm_Q_PopOne(manager, &one)) return;

    len = HostComm_BuildDataFrame(&one, s_tx_frame);
    if (len == 0u) return;

    (void)CDC_Transmit_FS(s_tx_frame, len);
}

/* ==================== 下行命令 ASCII ==================== */
void HostComm_Rx_Parse(HostComm_Manager_t *manager, uint8_t *buf, uint32_t len)
{
    char cmd[128];
    int32_t set_val;
    int wave_val;
    int stop_level;
    int32_t d_kp, d_ki, q_kp, q_ki;
    int32_t r_q16, l_q16, flux_q16, pp_q0;

    if (manager == NULL || manager->foc == NULL || buf == NULL || len == 0u) return;

    if (len >= sizeof(cmd)) len = sizeof(cmd) - 1u;
    memcpy(cmd, buf, len);
    cmd[len] = '\0';

    while (len > 0u) {
        char c = cmd[len - 1u];
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t') {
            cmd[len - 1u] = '\0';
            len--;
        } else break;
    }
    if (len == 0u) return;

    if (strncmp(cmd, "PI_PARA:", 8) == 0) {
        if (sscanf(cmd, "PI_PARA:%ld,%ld,%ld,%ld", &d_kp, &d_ki, &q_kp, &q_ki) == 4) {
            manager->foc->pid_d.Kp = d_kp;
            manager->foc->pid_d.Ki = d_ki;
            manager->foc->pid_q.Kp = q_kp;
            manager->foc->pid_q.Ki = q_ki;
            HostComm_CmdTouch();
            HostComm_SetAck("ACK:PI_PARA");
        } else {
            HostComm_SetAck("ERR:PI_PARA");
        }
        return;
    }

    if (strncmp(cmd, "M_PARA:", 7) == 0) {
        if (sscanf(cmd, "M_PARA:%ld,%ld,%ld,%ld", &r_q16, &l_q16, &flux_q16, &pp_q0) == 4) {
            (void)r_q16; (void)l_q16; (void)flux_q16; (void)pp_q0;
            HostComm_CmdTouch();
            HostComm_SetAck("ACK:M_PARA");
        } else {
            HostComm_SetAck("ERR:M_PARA");
        }
        return;
    }

    if (strncmp(cmd, "SET:", 4) == 0) {
        if (sscanf(cmd, "SET:%ld", &set_val) == 1) {

            if (manager->ctrl_mode == MODE_SPD) {
                if (set_val >  HOSTCOMM_SPD_SET_MAX_RPM) set_val =  HOSTCOMM_SPD_SET_MAX_RPM;
                if (set_val < -HOSTCOMM_SPD_SET_MAX_RPM) set_val = -HOSTCOMM_SPD_SET_MAX_RPM;
            } else if (manager->ctrl_mode == MODE_CURR) {
                if (set_val >  HOSTCOMM_CURR_SET_MAX_Q16) set_val =  HOSTCOMM_CURR_SET_MAX_Q16;
                if (set_val < -HOSTCOMM_CURR_SET_MAX_Q16) set_val = -HOSTCOMM_CURR_SET_MAX_Q16;
            } else {
                set_val = 0;
            }

            manager->setpoint = set_val;
            if (manager->ctrl_mode == MODE_CURR) {
                manager->foc->pid_q.Ref = set_val;
            }

            HostComm_CmdTouch();
            HostComm_SetAck("ACK:SET");
        } else {
            HostComm_SetAck("ERR:SET");
        }
        return;
    }

    if (strncmp(cmd, "WAVE:", 5) == 0) {
        if (sscanf(cmd, "WAVE:%d", &wave_val) == 1) {
            if (wave_val >= WAVE_PHASE_I && wave_val <= WAVE_ANGLE) {
                manager->wave_mode = (HostComm_WaveMode_e)wave_val;
                HostComm_Q_Clear(manager);
                HostComm_CmdTouch();
                HostComm_SetAck("ACK:WAVE");
            } else {
                HostComm_SetAck("ERR:WAVE_RANGE");
            }
        } else {
            HostComm_SetAck("ERR:WAVE_FMT");
        }
        return;
    }

    if (strncmp(cmd, "MODE:", 5) == 0) {
        if (strstr(cmd, "CURR")) {
            manager->ctrl_mode = MODE_CURR;
            manager->setpoint = 0;
            manager->foc->pid_d.Ref = 0;
            manager->foc->pid_q.Ref = 0;
            manager->foc->pid_d.Integral = 0;
            manager->foc->pid_q.Integral = 0;
            HostComm_CmdTouch();
            HostComm_SetAck("ACK:MODE_CURR");
        } else if (strstr(cmd, "SPD")) {
            manager->ctrl_mode = MODE_SPD;
            manager->setpoint = 0;
            manager->foc->pid_d.Ref = 0;
            manager->foc->pid_q.Ref = 0;
            manager->foc->pid_d.Integral = 0;
            manager->foc->pid_q.Integral = 0;
            HostComm_CmdTouch();
            HostComm_SetAck("ACK:MODE_SPD");
        } else if (strstr(cmd, "IDLE")) {
            HostComm_DoSoftStop(manager);
            HostComm_CmdTouch();
            HostComm_SetAck("ACK:MODE_IDLE");
        } else {
            HostComm_SetAck("ERR:MODE");
        }
        return;
    }

    if (strncmp(cmd, "STOP:", 5) == 0) {
        stop_level = 1;
        (void)sscanf(cmd, "STOP:%d", &stop_level);

        if (stop_level >= 2) {
            HostComm_DoHardStop(manager);
            HostComm_SetAck("ACK:STOP_HARD");
        } else {
            HostComm_DoSoftStop(manager);
            HostComm_SetAck("ACK:STOP_SOFT");
        }
        HostComm_CmdTouch();
        return;
    }

    HostComm_SetAck("ERR:UNKNOWN_CMD");
}
