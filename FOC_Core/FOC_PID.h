#ifndef __FOC_PID_H
#define __FOC_PID_H
#include "FOC_Qformat.h"
typedef struct {
    // ==================== 1. PID 增益系数 (Q16) ====================
    q16_t Kp;           // 比例增益 (Q16 格式)
    q16_t Ki;           // 积分增益 (Q16 格式)
    q16_t Kd;           // 微分增益 (Q16 格式)

    // ==================== 2. 限幅保护值 (Q16) ====================
    q16_t Out_Max;      // 输出上限 (Q16 格式，如 1.0 PU = 65536)
    q16_t Out_Min;      // 输出下限 (Q16 格式，如 -1.0 PU = -65536)

    // ==================== 3. 运行状态变量 (Q16) ====================
    q16_t Ref;          // 目标给定值 (Q16 标幺值)
    q16_t Fdb;          // 实际反馈值 (Q16 标幺值)
    q16_t Last_Fdb;     // 上一次反馈值 (Q16 标幺值，用于微分计算)
    q16_t Error;        // 偏差值 (Q16 标幺值)
    q16_t Integral;     // 积分累加器 (Q16 标幺值)
    q16_t Output;       // PID 输出值 (Q16 标幺值)
} PID_t;



 /** @brief  PID 控制器计算 (Q16 定点标幺化内联版)
 * @param  PID:            PID 结构体指针
 * @param  measured_value: 当前时刻反馈的实际物理量 (Q16 标幺值)
 */
static __inline void FS_PID_Calculate(PID_t *PID, q16_t measured_value) {
    // 1. 将高频使用的参数和变量一次性读入 CPU 通用寄存器
    int32_t kp      = PID->Kp;
    int32_t ki      = PID->Ki;
    int32_t kd      = PID->Kd;
    int32_t out_max = PID->Out_Max;
    int32_t out_min = PID->Out_Min;
    int32_t ref     = PID->Ref;
    int32_t last_fdb= PID->Last_Fdb;
    int32_t integral= PID->Integral;

    // 2. 计算偏差 Error = Ref - Fdb
    int32_t fdb   = measured_value;
    int32_t error = ref - fdb;

    // 3. 比例项计算: p_out = (Kp * Error) >> 16
    // 两个 Q16 乘积为 Q32 格式，为了防止 32 位溢出，强转为 64 位乘法，右移 16 位还原为 Q16
    int32_t p_out = (int32_t)(((int64_t)kp * error) >> 16);

    // 4. 积分项累加: Integral += (Ki * Error) >> 16
    int32_t i_step = (int32_t)(((int64_t)ki * error) >> 16);
    integral += i_step;

    // 5. 积分抗饱和限制 (Anti-windup)
    // 积分值不能超过输出的最大/最小边界
    if (integral > out_max) {
        integral = out_max;
    } else if (integral < out_min) {
        integral = out_min;
    }

    // 6. 微分项计算: d_out = Kd * (Last_Fdb - Fdb) >> 16
    // 完美保留你原本“基于测量值微分（Derivative on Measurement）”的优良特性，避免目标值突变时产生微分冲击
    int32_t diff_fdb = last_fdb - fdb;
    int32_t d_out = (int32_t)(((int64_t)kd * diff_fdb) >> 16);

    // 7. 更新结构体状态，保存当前时刻反馈值供下一次微分使用
    PID->Fdb      = fdb;
    PID->Error    = error;
    PID->Last_Fdb = fdb; 
    PID->Integral = integral;

    // 8. 汇总计算总输出: Output = p_out + Integral + d_out
    int32_t output = p_out + integral + d_out;

    // 9. 最终输出限幅
    if (output > out_max) {
        output = out_max;
    } else if (output < out_min) {
        output = out_min;
    }
    PID->Output = output;
}

#endif