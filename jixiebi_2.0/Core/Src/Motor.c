#include "Motor.h"

Motor_Speed_t motor_speed = {0.0f, 0.0f};
PID_Type TurnPID;
volatile float CurrentYaw = 0.0f;      // 当前总偏航角 (积分得到)

void Motor_Init_PWM(void) {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // 设置左轮 PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // 设置右轮 PWM
	PID_Init(&TurnPID, 1.5f, 0.1f, 0.25f, 80.0f, 40.0f);
}

void Motor_SetSpeed_PWM(float leftPwm, float rightPwm) {
    // 将浮点数转换为整数，用于寄存器写入
    int leftDuty = (int)leftPwm;
    int rightDuty = (int)rightPwm;
    
    if (leftDuty > 0) { // 前进
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); 
    } else if (leftDuty < 0) { // 后退
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        leftDuty = -leftDuty; // 取绝对值作为PWM占空比
    } else { // 停止
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    }
    // 限制PWM范围 [0, 1000]
    if(leftDuty > 900) leftDuty = 900;
    if(leftDuty < 0) leftDuty = 0;

	if (rightDuty > 0) { // 前进
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);  
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); 
    } else if (rightDuty < 0) { // 后退
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
        rightDuty = -rightDuty;
    } else { // 停止
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    // 限制PWM范围 [0, 1000]
    if(rightDuty > 900) rightDuty = 900;
    if(rightDuty < 0) rightDuty = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, leftDuty); // 设置左轮 PWM
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, rightDuty); // 设置右轮 PWM
}

Motor_Speed_t Motor_GetPWM(float TargetYaw) {
	float gyro_z = MPU_Get_Gyro_Z();
	CurrentYaw += gyro_z * 0.01f; 
    float turn_adjust = PID_Calculate(&TurnPID, TargetYaw, CurrentYaw);
    float err = TargetYaw - CurrentYaw;
    if (fabs(err) < 1.5f) {
        turn_adjust = 0.0f;
        TurnPID.Integral = 0.0f; // 清空积分防止过冲
    }
    else {
        // 启动补偿：给它一个最小力气 (25 RPM)，防止推不动
        if (turn_adjust > 0 && turn_adjust < 25.0f) turn_adjust = 35.0f;
        if (turn_adjust < 0 && turn_adjust > -25.0f) turn_adjust = -35.0f;
    }
    motor_speed.left_pwm  =  - turn_adjust*10;
    motor_speed.right_pwm =  turn_adjust*10;
    return motor_speed;
}
