#include "pca9685.h"
#include "math.h"

// 内部写寄存器辅助函数
void PCA9685_Write(uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDR, reg, 1, &data, 1, 10);
}

// 内部读寄存器辅助函数
uint8_t PCA9685_Read(uint8_t reg) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDR, reg, 1, &data, 1, 10);
    return data;
}

// 初始化 PCA9685
// freq: 舵机通常是 50Hz
void PCA9685_Init(float freq) {
    // 1. 复位
    PCA9685_Write(PCA9685_MODE1, 0x00);
    
    // 2. 计算预分频值
    // 公式来自手册: prescale = round(osc_clock / (4096 * update_rate)) - 1
    // 内部时钟 25MHz
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f);

    // 3. 设置频率 (必须先进入睡眠模式才能设置)
    uint8_t oldmode = PCA9685_Read(PCA9685_MODE1);
    uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    PCA9685_Write(PCA9685_MODE1, newmode); // go to sleep
    PCA9685_Write(PCA9685_PRESCALE, prescale); // set the prescaler
    PCA9685_Write(PCA9685_MODE1, oldmode);
    HAL_Delay(5);
    PCA9685_Write(PCA9685_MODE1, oldmode | 0xa1); // 开启自动递增
}

// 设置 PWM 占空比 (0-4095)
// num: 通道 0-15
void PCA9685_SetPWM(uint8_t num, uint16_t on, uint16_t off) {
    uint8_t data[4] = {0};
    data[0] = on;
    data[1] = on >> 8;
    data[2] = off;
    data[3] = off >> 8;
    // 连续写入4个寄存器
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDR, LED0_ON_L + 4 * num, 1, data, 4, 10);
}

// 设置舵机角度 (封装得更方便)
// num: 0-15
// angle: 0-180度
void PCA9685_SetServoAngle(uint8_t num, float angle) {
    // 约束角度
    if(angle < 0) angle = 0;
    if(angle > 180) angle = 180;

    // 映射角度到脉宽 (0-4095)
    // 0度 = 0.5ms, 180度 = 2.5ms, 周期 = 20ms
    // 0.5ms / 20ms * 4096 ~= 102
    // 2.5ms / 20ms * 4096 ~= 512
    float off_value = 102.0f + (angle / 180.0f) * (512.0f - 102.0f);
    
    PCA9685_SetPWM(num, 0, (uint16_t)off_value);
}
