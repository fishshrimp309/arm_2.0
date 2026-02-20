#include "bldc_driver.h"

float final_start_angle = 2.0f;
float final_target_angle = 0.0f;   // 你最终想让电机去的位置 (0 或 3.14)
float current_target_angle = 0.0f; // PID 当前正在实时追踪的中间目标
float speed_limit = 0.005f;  
// 速度控制参数：每次控制循环允许目标角度改变的最大值
// 假设循环 2ms (500Hz)，0.005 * 500 = 2.5 弧度/秒 (大约需要 1.2秒转完半圈)
// 这个值越小，转得越慢！

PID_Controller angle_pid;// 1. 实例化并初始化 PID 控制器
BLDC_Driver_t motor_driver;
AS5600_t encoder;

const int POLE_PAIRS = 7; 
float zero_electric_angle = 41.4052086;
float electrical_angle = 0.0f;
float mechanical_angle = 0.0f;

void BLDC_Init(BLDC_Driver_t *driver, TIM_HandleTypeDef *htim, float supply_voltage) {
    driver->htim = htim;
    driver->voltage_power_supply = supply_voltage;
    driver->voltage_limit = supply_voltage;

    HAL_TIM_PWM_Start(driver->htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(driver->htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(driver->htim, TIM_CHANNEL_3);
}

void BLDC_Enable(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(DRIVER_EN_GPIO_Port, DRIVER_EN_Pin, GPIO_PIN_SET);
}

void BLDC_Disable(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(DRIVER_EN_GPIO_Port, DRIVER_EN_Pin, GPIO_PIN_RESET);
}

// SVPWM 实现
void BLDC_SetVoltage(BLDC_Driver_t *driver, float Uq, float angle_el) {
    float Uout = Uq;
    
    if (Uout > driver->voltage_limit) Uout = driver->voltage_limit;
    if (Uout < -driver->voltage_limit) Uout = -driver->voltage_limit;
    
    // 归一化角度
    float angle = fmod(angle_el, 2.0f * _PI);
    if (angle < 0) angle += 2.0f * _PI;

    // 计算扇区
    int sector = (int)(angle / (_PI / 3.0f)) + 1;
    float T1 = _PI / 3.0f * (float)sector - angle;
    float T2 = angle - _PI / 3.0f * (float)(sector - 1);

    // 计算占空比 (SVPWM)
    float t1 = sinf(T1) * Uout / driver->voltage_power_supply;
    float t2 = sinf(T2) * Uout / driver->voltage_power_supply;
    float t0 = 1.0f - t1 - t2;

    float Ta, Tb, Tc;

    switch (sector) {
        case 1: Ta = t1 + t2 + t0/2; Tb = t2 + t0/2; Tc = t0/2; break;
        case 2: Ta = t1 + t0/2; Tb = t1 + t2 + t0/2; Tc = t0/2; break;
        case 3: Ta = t0/2; Tb = t1 + t2 + t0/2; Tc = t2 + t0/2; break;
        case 4: Ta = t0/2; Tb = t1 + t0/2; Tc = t1 + t2 + t0/2; break;
        case 5: Ta = t2 + t0/2; Tb = t0/2; Tc = t1 + t2 + t0/2; break;
        case 6: Ta = t1 + t2 + t0/2; Tb = t0/2; Tc = t1 + t0/2; break;
        default: Ta = 0; Tb = 0; Tc = 0; // Error
    }

    // 写入CCR寄存器 (注意ARR=3599)
    __HAL_TIM_SET_COMPARE(driver->htim, TIM_CHANNEL_1, (uint16_t)(Ta * 3599));
    __HAL_TIM_SET_COMPARE(driver->htim, TIM_CHANNEL_2, (uint16_t)(Tb * 3599));
    __HAL_TIM_SET_COMPARE(driver->htim, TIM_CHANNEL_3, (uint16_t)(Tc * 3599));
}

void Motor_GotoAngle(float final_target_angle) {
	float angle_diff = final_target_angle - current_target_angle;
    // 核心：将差值限制在 [-PI, PI] 之间
    if (angle_diff > 3.14159265f)  angle_diff -= 2.0f * 3.14159265f;
    if (angle_diff < -3.14159265f) angle_diff += 2.0f * 3.14159265f;
    if (fabs(angle_diff) > speed_limit) {
        if (angle_diff > 0) {
            current_target_angle += speed_limit; // 顺时针近
        } else {
            current_target_angle -= speed_limit; // 逆时针近
        }
    } else {
        current_target_angle = final_target_angle;
    }
      mechanical_angle = AS5600_GetAngle(&encoder);    
      if (mechanical_angle >= 0) { 
          float Uq = PID_Compute(&angle_pid, current_target_angle, mechanical_angle);
          // 5. FOC 核心代码
          electrical_angle = (mechanical_angle * POLE_PAIRS) - zero_electric_angle;
          float drive_angle = electrical_angle + (3.14159265f / 2.0f);
          
          BLDC_SetVoltage(&motor_driver, Uq, drive_angle);
      }
}

void align_sensor(float final_start_angle) {
	current_target_angle = AS5600_GetAngle(&encoder);
	mechanical_angle = AS5600_GetAngle(&encoder);
    while (fabs(mechanical_angle - final_start_angle) > 0.01f) {
        Motor_GotoAngle(final_start_angle);
        HAL_Delay(1);
    }
}
