#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

#include <math.h>
#include "main.h"
#include "as5600.h"
#include "pid.h"

#define _PI 3.14159265359f

// 结构体 电机状态
typedef struct {
    TIM_HandleTypeDef *htim; // PWM定时器句柄
    uint16_t voltage_power_supply; // 供电电压
    float voltage_limit;     // 电压限制
} BLDC_Driver_t;


void BLDC_Init(BLDC_Driver_t *driver, TIM_HandleTypeDef *htim, float supply_voltage);// 初始化
void BLDC_Enable(void);//使能
void BLDC_Disable(void);//失能
void BLDC_SetVoltage(BLDC_Driver_t *driver, float Uq, float angle_el);// 设置电压矢量
void Motor_GotoAngle(float final_target_angle);
void align_sensor(float final_start_angle);

#endif
