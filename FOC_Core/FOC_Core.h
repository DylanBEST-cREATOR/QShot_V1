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
	   LPF_t lpfi_d;
     LPF_t lpfi_q;
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
extern void FC_SpeedLoop_Execute(uint16_t current_angle);
extern void FC_SpeedLoop_Reset(uint16_t current_angle);
void FC_FOC_OpenLoop_Rotate(FOC_Core_t *core, q16_t v_mag, q16_t angle_step);
extern  void FC_SpeedControl_Init(PID_t *pid, LPF_t *lpf);

/**
 * @brief  计算机械转速 RPM (Q16)
 * @param  current_raw: 当前编码器原始值 (0~4095)
 * @param  last_raw:    上次编码器原始值
 * @return q16_t: 实际转速 RPM (Q16)
 */
static __inline  q16_t AS5600_Calculate_RPM(uint16_t current_raw, uint16_t last_raw) {
    int32_t delta = (int32_t)current_raw - (int32_t)last_raw;

    // 处理 0-4095 临界点跳变
    if (delta > 2048)  delta -= 4096;
    if (delta < -2048) delta += 4096;

    // 转换为 RPM (Q16)
    // 使用 int64 确保中间结果不溢出
    return (q16_t)(((int64_t)delta * SPEED_CAL_COEFF));
}
/**
 * @brief  计算机械转速 RPM (修正为纯 Q16 标尺)
 */
//static __inline  q16_t AS5600_Calculate_RPM(uint16_t current_raw, uint16_t last_raw) {
//    int32_t delta = (int32_t)current_raw - (int32_t)last_raw;

//    // 处理 0-4095 临界点跳变
//    if (delta > 2048)  delta -= 4096;
//    if (delta < -2048) delta += 4096;

//    // 注意：delta(Q0) * SPEED_CAL_COEFF(Q16) = 结果直接就是 Q16 格式的 RPM
//    // 这里绝对不能再右移 16 位，否则会损失 65536 倍精度
//    return (q16_t)((int64_t)delta * SPEED_CAL_COEFF); 
//}

/**
 * @brief  独立速度控制器
 * @param  PID: 速度环专用 PID 结构体指针
 * @param  target_rpm: 目标转速 (Q16)
 * @param  fdb_rpm:    实际转速 (Q16)
 * @return q16_t: 输出的 Iq 指令电流 (Q16)
 */
static __inline q16_t FC_Speed_Loop_Execute(PID_t *PID, q16_t target_rpm, q16_t fdb_rpm) {
    // 1. 更新目标值
    PID->Ref = target_rpm;

    // 2. 调用通用的 PID 计算函数 (你之前写的 FS_PID_Calculate)
    FS_PID_Calculate(PID, fdb_rpm);

    // 3. 返回计算结果 (这个结果通常是 Iq 的目标值)
    return PID->Output;
}

#endif /* __FOC_CORE_H */
