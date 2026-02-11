#ifndef __PCA9685_H
#define __PCA9685_H

#include "main.h"
#include "i2c.h" // 确保包含 I2C 头文件

// PCA9685 默认地址 (0x40)，HAL库需要左移一位
// #define PCA9685_ADDR  0x40
#define PCA9685_ADDR  (0x40 << 1)

// 寄存器定义
#define PCA9685_MODE1 	0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 		0x06

// 函数声明
void PCA9685_Init(float freq);
void PCA9685_SetPWM(uint8_t num, uint16_t on, uint16_t off);
void PCA9685_SetServoAngle(uint8_t num, float angle);

#endif
