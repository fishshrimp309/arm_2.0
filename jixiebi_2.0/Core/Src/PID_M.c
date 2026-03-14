#include "PID_M.h"
#include <math.h>

void PID_Init(PID_Type *pid, float kp, float ki, float kd, float max_output, float max_integral)
{
    // 初始化PID参数
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->OutputMax = max_output;
    pid->IntegralMax = max_integral;
    
    // 初始化状态变量
    pid->TargetSpeed = 0.0f;
    pid->ActualSpeed = 0.0f;
    pid->LastError = 0.0f;
    pid->Integral = 0.0f;
}

float PID_Calculate(PID_Type *pid, float target_speed, float actual_speed)
{
    // 1. 计算误差
    pid->TargetSpeed = target_speed;
    pid->ActualSpeed = actual_speed;
    pid->Error = pid->TargetSpeed - pid->ActualSpeed;
    
    // 2. 比例项 (P)
    float P_Term = pid->Kp * pid->Error;

    // 3. 积分项 (I)
    pid->Integral += pid->Error;
    
    // 积分限幅 (重要！把这个限制改小一点，比如 200-300)
    if (pid->Integral > pid->IntegralMax) pid->Integral = pid->IntegralMax;
    if (pid->Integral < -pid->IntegralMax) pid->Integral = -pid->IntegralMax;
    
    float I_Term = pid->Ki * pid->Integral;

    // 4. 微分项 (D) 
    float D_Term = pid->Kd * (pid->Error - pid->LastError);
    
    // 5. 计算输出
    pid->PwmOutput = P_Term + I_Term + D_Term;
    
    // 6. 输出限幅
    if (pid->PwmOutput > pid->OutputMax) pid->PwmOutput = pid->OutputMax;
    if (pid->PwmOutput < -pid->OutputMax) pid->PwmOutput = -pid->OutputMax;

    // 7. 更新历史
    pid->LastError = pid->Error;

    return pid->PwmOutput;
}
