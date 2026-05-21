#include "FOC_Core.h"
#include <stdbool.h>
#include <stddef.h>
/**
 * @brief  FOC 核心控制结构体初始化
 * @param  core:  FOC核心结构体指针
 * @param  t_pwm: 定时器 PWM 周期值 (如 3500)
 */
 
 
 static uint16_t last_raw_angle = 0; 
static bool first_run = true;

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
core->pid_d.Kp = 84000;  // 
core->pid_d.Ki = 6000;   // 
core->pid_d.Kd = 0;

core->pid_d.Out_Max   =  CURRENT_PID_OUT_MAX ;      // 维持 SVPWM 最大矢量模长
core->pid_d.Out_Min   = - CURRENT_PID_OUT_MAX;
    core->pid_d.Ref       = 0;
    core->pid_d.Fdb       = 0;
    core->pid_d.Last_Fdb  = 0;
    core->pid_d.Error     = 0;
    core->pid_d.Integral  = 0;
    core->pid_d.Output    = 0;

    // ==================== 6. 初始化 q 轴 PID 调节器 ====================
// ==================== 重新校准后的 q 轴 PID 参数 (带宽 2kHz) ====================
// ==================== 极低增益测试版 ====================
core->pid_q.Kp = 84000;  // 恢复到接近原值
core->pid_q.Ki = 6000;   // 远大于你现在的 2000
core->pid_q.Kd = 0;
core->pid_q.Out_Max   = CURRENT_PID_OUT_MAX;       // 维持 SVPWM 最大矢量模长
core->pid_q.Out_Min   = - CURRENT_PID_OUT_MAX;
    core->pid_q.Ref       = 0;
    core->pid_q.Fdb       = 0;
    core->pid_q.Last_Fdb  = 0;
    core->pid_q.Error     = 0;
    core->pid_q.Integral  = 0;
    core->pid_q.Output    = 0;

    // ==================== 7. 初始化 SVPWM 模块 ====================
    core->svpwm.U_alpha   = 0;
    core->svpwm.U_beta    = 0;
    core->svpwm.U_dc      = 65536; //12v
    core->svpwm.T_pwm     = t_pwm;
    core->svpwm.Duty_A    = 0;
    core->svpwm.Duty_B    = 0;
    core->svpwm.Duty_C    = 0;
		core->lpfi_d.Alpha = 65536; // 0.1 in Q16
    core->lpfi_q.Alpha = 65536;
    core->lpfi_d.LastOutput = 0;
    core->lpfi_q.LastOutput = 0;
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
//    core->svpwm.U_dc = FS_Convert_Vbus_To_PU(core->adc_vbus_raw);
    core->svpwm.U_dc = 65536;
    // 3. 计算当前电角度对应的正余弦值 sin(theta) & cos(theta)
    FM_fastSinCos(core->angle, &core->sincos);

    // 4. Clarke 变换：(Ia, Ib, Ic) -> (Ialpha, Ibeta)
    FT_Clarke_Transform(&core->current, &core->clarke_i);

    // 5. Park 变换：(Ialpha, Ibeta) -> (Id, Iq)
    FT_Park_Transform(&core->clarke_i, &core->sincos, &core->park_i);
		core->park_i.d = FM_LPF_Calculate(&core->lpfi_d, core->park_i.d);
    core->park_i.q = FM_LPF_Calculate(&core->lpfi_q, core->park_i.q);

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

/**
 * @brief  FOC 开环旋转测试函数
 * @param  core: FOC核心结构体指针
 * @param  v_mag: 给定的电压模长 (Q16, 建议取 3000~8000，即约 5%~12% PU)
 * @param  angle_step: 每周期角度增量 (Q16, 决定了转速。例如 20kHz 下 65 代表约 1Hz 电频率)
 */
void FC_FOC_OpenLoop_Rotate(FOC_Core_t *core, q16_t v_mag, q16_t angle_step) {
    if (core == NULL) return;

    // 1. 角度自增 (Q16 范围控制在 [-65536, 65536] 即 [-PI, PI])
    core->angle += angle_step;
    
    // 角度回绕处理
    if (core->angle > Q16_PI)  core->angle -= Q16_2PI;
    if (core->angle < -Q16_PI) core->angle += Q16_2PI;

    // 2. 更新母线电压 (确保 SVPWM 计算比例正确)
//    core->svpwm.U_dc = FS_Convert_Vbus_To_PU(core->adc_vbus_raw);
    core->svpwm.U_dc = 65536;
    // 3. 生成旋转的 V_alpha 和 V_beta
    // 使用你已有的快速正余弦函数
    FM_fastSinCos(core->angle, &core->sincos);

    // V_alpha = V_mag * cos(theta)
    // V_beta  = V_mag * sin(theta)
    core->svpwm.U_alpha = (q16_t)(((int64_t)v_mag * core->sincos.c) >> 16);
    core->svpwm.U_beta  = (q16_t)(((int64_t)v_mag * core->sincos.s) >> 16); // 注意：这里应改为 sincos.s

    /* 纠正上面的笔误，代码应如下： */
    core->svpwm.U_alpha = _Q16mpy(v_mag, core->sincos.c);
    core->svpwm.U_beta  = _Q16mpy(v_mag, core->sincos.s);

    // 4. 调用你已有的 SVPWM 计算函数
    FS_SVPWM_Calculate(&core->svpwm);
}

/**
 * @brief  速度环控制系统初始化
 * @param  pid: 速度环 PID 结构体指针
 * @param  lpf: 速度反馈滤波器结构体指针
 */
void FC_SpeedControl_Init(PID_t *pid, LPF_t *lpf) {
    // ==================== 1. 速度 PID 参数配置 ====================

    pid->Kp = 8000;    
    pid->Ki = 100;     
    pid->Kd = 0;      

    // 关键：速度环的输出限幅 = 电流环允许的最大 Iq 电流
    // 使用你 APP_PARAMS_H 中定义的 SPEED_PID_OUT_MAX (比如 1.5A)
    pid->Out_Max = SPEED_PID_OUT_MAX; 
    pid->Out_Min = -SPEED_PID_OUT_MAX;

    // 清零内部状态变量
    pid->Ref      = 0;
    pid->Fdb      = 0;
    pid->Error    = 0;
    pid->Integral = 0;
    pid->Output   = 0;
    pid->Last_Fdb = 0;

    // ==================== 2. 速度反馈滤波器配置 ====================
    // AS5600 噪声较大，Alpha 值建议设置在 0.05 ~ 0.2 之间
    // Alpha 越小，滤波越强，但延迟越高
    lpf->Alpha = 3277;     // 0.1 * 65536
    lpf->LastOutput = 0;
}
