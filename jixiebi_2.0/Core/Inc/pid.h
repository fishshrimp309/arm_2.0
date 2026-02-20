#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "main.h"

// PID 结构体定义
typedef struct {
    // PID 参数
    float Kp;
    float Ki;
    float Kd;

    // 状态变量
    float integral;      // 误差积分累积
    float error_prev;    // 上一次的误差 (用于微分计算)

    // 输出限幅
    float output_limit;  // 最大输出限制 (防止电压过大烧毁电机)
} PID_Controller;

// PID 函数声明
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float limit);
float PID_Compute(PID_Controller *pid, float target, float current);
void PID_Reset(PID_Controller *pid); // 用于清空积分和历史误差

#endif // PID_H
