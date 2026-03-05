#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

void MPU_Init(void);
void MPU_Calibrate_Gyro_Z(uint16_t samples);
float MPU_Get_Gyro_Z(void);

#endif
