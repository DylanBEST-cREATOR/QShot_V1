#ifndef __FOC_CORE_H
#define __FOC_CORE_H



#define _round(x) ((x)>=0?(long)((x)+0.5f):(long)((x)-0.5f))
        
#define PID_CURRENT_OPTION
#define PID_SPEED_OPTION

#define VOLTS_TO_AMPS_RATIO  2.0f  







typedef struct {
    float U_alpha;      // 输入 Alpha 轴电压
    float U_beta;       // 输入 Beta 轴电压
    float U_dc;         // 母线电压
    float T_pwm;        // PWM 周期（或设置为 1.0 用于输出占空比）
    
    
    float Duty_A;
    float Duty_B;
    float Duty_C;
} SVPWM_t;

typedef struct {
    
    float Kp;
    float Ki;
    float Kd;

    
    float Out_Max;
    float Out_Min;

    // 变量
    float Ref;        
    float Fdb;        
    float Last_Fdb;    
    float Error;       
    float Integral;    
    float Output;
} PID_t;

// Clark 变换对象
typedef struct {
    float I_a;  
    float I_b;        
    float I_c;     
    float I_alpha; 
    float I_beta;   
} Clark_t;

// Park 变换对象
typedef struct {
    float I_alpha;  // Input: stationary alpha-axis
    float I_beta;   // Input: stationary beta-axis
    float theta;  // Input: rotor angle (0~2PI)
    float I_d;      // Output: rotating d-axis
    float I_q;      // Output: rotating q-axis
} Park_t;

// 反 Park 变换对象
typedef struct {
    float d;      // Input: rotating d-axis
    float q;      // Input: rotating q-axis
    float theta;  // Input: rotor angle (0~2PI)
    float U_alpha;  // Output: stationary alpha-axis
    float U_beta;   // Output: stationary beta-axis
} Ip_t;


typedef struct {
    float theta;            // 输入：电角度 (0~2PI)
    float Target_Id;        // 输入：D轴目标电流
    float Target_Iq;        // 输入：Q轴目标电流

    // 算法子模块实例
    Clark_t  Clark;         // Clark 变换
    Park_t   Park;          // Park 变换
    Ip_t     InvPark;       // 反 Park 变换
    SVPWM_t  SVPWM;         // SVPWM 模块
    
    PID_t    PidId;         // D 轴电流 PID
    PID_t    PidIq;         // Q 轴电流 PID
} FOC_t;


extern float AL_electricalAngle(float shaft_angle, int pole_pairs);
extern float AL_normalizeAngle(float angle);
//extern static __inline void AL_Clark(Clark_t *CLARK);
//extern static __inline void AL_InvPark(Ip_t *IPARK, float sin_t, float cos_t);
//extern static __inline void AL_Park(Park_t *PARK, float sin_t, float cos_t);
extern void Al_FOCCalc_Init(volatile FOC_t *FOC);
extern void Al_FOC_Calculate(volatile FOC_t *FOC);


#endif