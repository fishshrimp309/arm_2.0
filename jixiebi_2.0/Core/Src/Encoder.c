#include "Encoder.h"

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);//右
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);//左
}

float Encoder_Get_RPM(TIM_HandleTypeDef *htim)
{
    // 1. 获取计数值 (编码器模式下是带符号的增量值)
    // 强制转换为 int16_t 以处理负值 (方向)
    int16_t current_count = (int16_t)__HAL_TIM_GET_COUNTER(htim);
    
    // 2. 重置计数器，以便计算下一次的增量
    __HAL_TIM_SET_COUNTER(htim, 0); 
    
    // 3. 计算 Counts/Sec
    float counts_per_sec = (float)current_count / SAMPLE_TIME_S;
    
    // 4. 将 Counts/Sec 转换为 RPM (转/分钟)
    // 公式: RPM = (Counts/Sec * 60) / CPR 
    float rpm = (counts_per_sec * 60.0f) / MOTOR_ENCODER_CPR;
    
    return rpm;
}
