#include "FOC_Core.h"


#include "FOC_Core.h"
#include <stddef.h>

/**
 * @brief  FOC 核心控制结构体初始化
 * @param  core:  FOC核心结构体指针
 * @param  t_pwm: 定时器 PWM 周期值 (如 3500)
 */
void FC_Core_Init(FOC_Core_t *core, uint32_t t_pwm) {
    if (core == NULL) {
        return;
    }

    // ==================== 1. 初始化系统级实时输入 ====================
    core->angle = 0;
    core->adc_vbus_raw = 0;

    // ==================== 2. 初始化电流采样模块 ====================
    // 硬件偏置初始化默认设为 12位 ADC 的中点 (2048)。
    // 提示：你可以在启动电机前，通过硬件校准函数重新覆盖这三个 offset 值。
    core->current.offset_1 = 0;  //此相不用
    core->current.offset_2 = 2048;
    core->current.offset_3 = 2048;     
    
    core->current.adc_raw_1 = 0;
    core->current.adc_raw_2 = 0;
    core->current.adc_raw_3 = 0;
    
    core->current.Ia = 0;
    core->current.Ib = 0;
    core->current.Ic = 0;

    // ==================== 3. 初始化正余弦模块 ====================
    core->sincos.s = 0;
    core->sincos.c = 65536; // cos(0) = 1.0 (Q16)

    // ==================== 4. 清空坐标变换中间状态 ====================
    core->clarke_i.alpha = 0;
    core->clarke_i.beta  = 0;
    
    core->park_i.d       = 0;
    core->park_i.q       = 0;
    
    core->park_v.d       = 0;
    core->park_v.q       = 0;
    
    core->clarke_v.alpha = 0;
    core->clarke_v.beta  = 0;

    // ==================== 5. 初始化 d 轴 PID 调节器 ====================
    // 默认输出电压限幅设为 [-37837, 37837] (即约 ±0.577 PU，对应不失真最大线性输出 1/sqrt(3))
    core->pid_d.Kp        = 0;
    core->pid_d.Ki        = 0;
    core->pid_d.Kd        = 0;
    core->pid_d.Out_Max   = 37837;  // 1/sqrt(3) 的 Q16 定点数
    core->pid_d.Out_Min   = -37837; 
    core->pid_d.Ref       = 0;
    core->pid_d.Fdb       = 0;
    core->pid_d.Last_Fdb  = 0;
    core->pid_d.Error     = 0;
    core->pid_d.Integral  = 0;
    core->pid_d.Output    = 0;

    // ==================== 6. 初始化 q 轴 PID 调节器 ====================
    core->pid_q.Kp        = 0;
    core->pid_q.Ki        = 0;
    core->pid_q.Kd        = 0;
    core->pid_q.Out_Max   = 37837;  // 1/sqrt(3) 的 Q16 定点数
    core->pid_q.Out_Min   = -37837;
    core->pid_q.Ref       = 0;
    core->pid_q.Fdb       = 0;
    core->pid_q.Last_Fdb  = 0;
    core->pid_q.Error     = 0;
    core->pid_q.Integral  = 0;
    core->pid_q.Output    = 0;

    // ==================== 7. 初始化 SVPWM 模块 ====================
    core->svpwm.U_alpha   = 0;
    core->svpwm.U_beta    = 0;
    core->svpwm.U_dc      = 65536; // 默认为 24.0V 基准电压 (1.0 PU, 对应 V_BASE_Q16)
    core->svpwm.T_pwm     = t_pwm;
    core->svpwm.Duty_A    = 0;
    core->svpwm.Duty_B    = 0;
    core->svpwm.Duty_C    = 0;
}

/**
 * @brief  FOC 核心控制算法单次执行函数
 * @param  core: FOC核心结构体指针
 */
void FC_FOC_Core(FOC_Core_t *core) {
    if (core == NULL) {
        return;
    }

    // 1. 电流采样值转换：将 ADC 原始值翻译并标幺化为 Ia, Ib, Ic (Q16)
    FS_ADCValueToCurrent(&core->current);

    // 2. 母线电压更新：转换母线电压 ADC 原始值为 Q16 标幺值，更新至 SVPWM
    core->svpwm.U_dc = FS_Convert_Vbus_To_PU(core->adc_vbus_raw);

    // 3. 计算当前电角度对应的正余弦值 sin(theta) & cos(theta)
    FM_fastSinCos(core->angle, &core->sincos);

    // 4. Clarke 变换：(Ia, Ib, Ic) -> (Ialpha, Ibeta)
    FT_Clarke_Transform(&core->current, &core->clarke_i);

    // 5. Park 变换：(Ialpha, Ibeta) -> (Id, Iq)
    FT_Park_Transform(&core->clarke_i, &core->sincos, &core->park_i);

    // 6. 电流环 PI 调节：
    // 注意：外部速度环或其他控制策略在调用此函数前，应先更新 core->pid_d.Ref 和 core->pid_q.Ref
    FS_PID_Calculate(&core->pid_d, core->park_i.d);
    FS_PID_Calculate(&core->pid_q, core->park_i.q);

    // 将 PI 调节器的输出赋给电压旋转坐标 (Vd, Vq)
    core->park_v.d = core->pid_d.Output;
    core->park_v.q = core->pid_q.Output;

    // 7. 反 Park 变换：(Vd, Vq) -> (Valpha, Vbeta)
    FT_InvPark_Transform(&core->park_v, &core->sincos, &core->clarke_v);

    // 8. SVPWM 占空比计算：(Valpha, Vbeta) -> (Duty_A, Duty_B, Duty_C)
    core->svpwm.U_alpha = core->clarke_v.alpha;
    core->svpwm.U_beta  = core->clarke_v.beta;
    FS_SVPWM_Calculate(&core->svpwm);
}

