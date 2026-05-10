#ifndef __FOC_CORE_H
#define __FOC_CORE_H
#include "FOC_Math.h"
#include "FOC_Transforms.h"
#include "FOC_Svpwm.h"
#include "FOC_PID.h"
#include "FOC_Observer.h"



/**
 * @brief FOC 核心控制总结构体
 * @note  该结构体统一管理一个 FOC 电机实例的所有实时状态、采样输入与控制输出。
 */
typedef struct {
    // ==================== 1. 系统级实时输入 (每次执行 FOC 前必须更新) ====================
    q16_t angle;                // 实时电角度标幺值 (Q16格式, [-65536, 65536] 对应 [-pi, pi])
    uint16_t adc_vbus_raw;      // 母线电压 ADC 原始采样值 (Q0 格式)

    // ==================== 2. 子算法模块状态管理 ====================
    Current_Sense_t current;    // 电流采样与相电流重建模块
    SinCos_t sincos;            // 快速正余弦计算结果存储
    Clarke_t clarke_i;          // 电流 Clarke 变换输出 (Ialpha, Ibeta)
    Park_t park_i;              // 电流 Park 变换输出 (Id, Iq)
    PID_t pid_d;                // d 轴电流 PI 调节器
    PID_t pid_q;                // q 轴电流 PI 调节器
    Park_t park_v;              // 反 Park 变换输入电压 (Vd, Vq)
    Clarke_t clarke_v;          // 反 Park 变换输出电压 (Valpha, Vbeta)
    SVPWM_t svpwm;              // SVPWM 计算模块（含 A/B/C 三相占空比输出）
} FOC_Core_t;

/* ==================== 核心函数接口 ==================== */

/**
 * @brief  FOC 核心控制结构体初始化
 * @param  core:  FOC核心结构体指针
 * @param  t_pwm: 定时器 PWM 周期（即自动重装载寄存器 ARR 的值，如 3500）
 */
void FC_Core_Init(FOC_Core_t *core, uint32_t t_pwm);

/**
 * @brief  FOC 核心控制算法单次执行函数 (电流环核心)
 * @param  core:  FOC核心结构体指针
 * @note   通常在 ADC 采样完成中断或定时器溢出中断中调用。
 */
void FC_FOC_Core(FOC_Core_t *core);

#endif /* __FOC_CORE_H */
