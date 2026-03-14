#include "mpu6050.h"
#include "i2c.h"  // 必须包含此文件以获取 hi2c2 定义
#include <math.h>

/* MPU6050 寄存器地址 */
#define MPU_ADDR (0x68 << 1) // AD0接地时地址为0x68，左移一位用于HAL库
#define PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1B
#define GYRO_ZOUT_H 0x47

// 零点漂移校准值
float Gyro_Z_Offset = 0.0f;

/**
 * @brief  初始化 MPU6050 (使用 I2C2)
 */
void MPU_Init(void) {
    uint8_t check;
    uint8_t data;

    // 1. 检查设备 ID (WHO_AM_I)
    if (HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, 0x75, 1, &check, 1, 100) == HAL_OK) {
        if (check == 0x68) {
            // 2. 唤醒 MPU6050 (解除休眠)
            data = 0x00;
            HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, PWR_MGMT_1, 1, &data, 1, 100);
            
            // 3. 配置陀螺仪量程 +/- 2000 dps (0x18)
            // 越大的量程越不容易溢出，但精度稍低，对于小车足够
            data = 0x18; 
            HAL_I2C_Mem_Write(&hi2c2, MPU_ADDR, GYRO_CONFIG, 1, &data, 1, 100);
        }
    }
}

/**
 * @brief  校准 Z 轴零点漂移
 * @param  samples: 采样次数 (建议 1000 次)
 * @note   调用此函数时，车必须绝对静止！
 */
void MPU_Calibrate_Gyro_Z(uint16_t samples) {
    long sum = 0;
    int16_t raw_z;
    uint8_t buf[2];
    
    for(int i=0; i<samples; i++) {
        HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, GYRO_ZOUT_H, 1, buf, 2, 10);
        raw_z = (int16_t)(buf[0] << 8 | buf[1]);
        sum += raw_z;
        HAL_Delay(2); // 稍微延时
    }
    Gyro_Z_Offset = (float)sum / samples;
}

/**
 * @brief  读取 Z 轴角速度
 * @retval 角速度 (单位: 度/秒 deg/s)
 */
float MPU_Get_Gyro_Z(void) {
    uint8_t buf[2];
    int16_t raw_z;
    float gyro_dps;

    // 读取高低8位
    if(HAL_I2C_Mem_Read(&hi2c2, MPU_ADDR, GYRO_ZOUT_H, 1, buf, 2, 5)!=HAL_OK){
		return 0.0f;
	}
    raw_z = (int16_t)(buf[0] << 8 | buf[1]);

    // 减去零偏
    float raw_float = (float)raw_z - Gyro_Z_Offset;

    // 转换公式: 量程2000dps 对应灵敏度 16.4 LSB/(deg/s)
    gyro_dps = raw_float / 16.4f; 
    
    // 简单的死区过滤，消除静止时的微小跳动
    if(fabs(gyro_dps) < 1.0f) gyro_dps = 0.0f;

    return gyro_dps; 
}
