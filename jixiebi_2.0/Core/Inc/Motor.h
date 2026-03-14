#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"
#include "tim.h"
#include "PID_M.h"
#include "math.h"
#include "mpu6050.h"

typedef struct {
    float left_pwm;
    float right_pwm;
} Motor_Speed_t;

void Motor_Init_PWM(void); 
void Motor_SetSpeed_PWM(float leftPwm, float rightPwm); 
Motor_Speed_t Motor_GetPWM(float TargetYaw);

#endif /* __MOTOR_H */
