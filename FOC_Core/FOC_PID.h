#ifndef __FOC_PID_H
#define __FOC_PID_H
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
#endif