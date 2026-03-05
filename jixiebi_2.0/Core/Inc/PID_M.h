#ifndef __PID_M_H
#define __PID_M_H

#include "main.h"

// PID结构体定义
typedef struct {
    float TargetSpeed; // 目标速度 (e.g., RPM)
    float ActualSpeed; // 实际速度
    float Kp, Ki, Kd;  // PID 系数
    
    // 内部计算变量
    float Error;
    float LastError;
    float PwmOutput;   // PID输出值 (对应-1000.0到1000.0的PWM)
    float Integral;
    
    float OutputMax;   // 输出限幅 (例如 1000.0)
    float IntegralMax; // 积分限幅
} PID_Type;

// 函数声明
void PID_Init(PID_Type *pid, float kp, float ki, float kd, float max_output, float max_integral);
float PID_Calculate(PID_Type *pid, float target_speed, float actual_speed);

#endif /* __PID_M_H */
