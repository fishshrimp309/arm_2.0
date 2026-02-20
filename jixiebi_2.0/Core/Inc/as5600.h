#ifndef AS5600_H
#define AS5600_H

#include "main.h"

#define AS5600_ADDR 0x36 // I2C地址 (7位地址左移一位)
#define AS5600_RAW_ANGLE_REG 0x0C

typedef struct {
    I2C_HandleTypeDef *hi2c;
    float angle_prev;
    int32_t full_rotations;
} AS5600_t;

void AS5600_Init(AS5600_t *sensor, I2C_HandleTypeDef *hi2c);
float AS5600_GetAngle(AS5600_t *sensor); // 返回弧度 0 ~ 2PI

#endif
