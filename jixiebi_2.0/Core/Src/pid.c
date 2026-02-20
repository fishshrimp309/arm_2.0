#include "pid.h"

// 初始化 PID 参数
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float limit) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->output_limit = limit;
    
    // 初始化状态变量
    pid->integral = 0.0f;
    pid->error_prev = 0.0f;
}

// 重置 PID 状态（当电机关闭或重新启动时调用，防止积分突变）
void PID_Reset(PID_Controller *pid) {
    pid->integral = 0.0f;
    pid->error_prev = 0.0f;
}

// 计算 PID 输出
// 计算 PID 输出
float PID_Compute(PID_Controller *pid, float target, float current) {
    // 1. 计算当前误差
    float error = target - current;

    // ==========================================
    // 核心修复：角度最短路径处理 (限制误差在 -PI 到 +PI 之间)
    // ==========================================
    if (error > 3.14159265f) {
        error -= 6.2831853f; // 6.2831853f 是 2*PI
    } else if (error < -3.14159265f) {
        error += 6.2831853f;
    }

    // 2. 积分计算 (带简单的抗积分饱和)
    pid->integral += error;
    float max_integral = pid->output_limit / (pid->Ki + 0.0001f);
    if (pid->Ki > 0.0f) {
        if (pid->integral > max_integral) pid->integral = max_integral;
        if (pid->integral < -max_integral) pid->integral = -max_integral;
    }

    // 3. 微分计算
    float derivative = error - pid->error_prev;
    pid->error_prev = error; // 更新误差

    // 4. 计算总输出
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 5. 输出限幅
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }

    return output;
}
