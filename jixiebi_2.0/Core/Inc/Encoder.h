#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "tim.h" 

#define ENCODER_PPR  13
#define GEAR_RATIO   20.0f //减速比
#define MOTOR_ENCODER_CPR  (ENCODER_PPR * 4.0f * GEAR_RATIO) 
#define SAMPLE_TIME_S 0.01f // PID 采样时间 (TIM1 的周期: 10ms = 0.01s)

void Encoder_Init(void);
float Encoder_Get_RPM(TIM_HandleTypeDef *htim); // 获取电机轴转速 (单位: r/min - RPM)

#endif /* __ENCODER_H */
