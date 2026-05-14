#include "FOC_Math.h"

void FM_VirtualAngle_SetSpeed(VirtualAngle_t *v_angle, int32_t target_rpm, uint8_t pole_pairs) {
    // 1. 计算电频率 Hz = (RPM * 极对数) / 60
    // 为了保持精度，先乘后除
    int32_t elec_freq = (target_rpm * pole_pairs) / 60;
    
    // 2. 计算步长 Step = (elec_freq * 2*PI) / Ctrl_Freq
    // 2*PI 在你的 Q16 定义中是 131072
    v_angle->Angle_Step = (q16_t)(((int64_t)elec_freq * 131072) / v_angle->Ctrl_Freq_Hz);
}


