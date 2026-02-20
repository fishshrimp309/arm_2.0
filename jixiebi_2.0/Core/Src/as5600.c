#include "as5600.h"

void AS5600_Init(AS5600_t *sensor, I2C_HandleTypeDef *hi2c) {
    sensor->hi2c = hi2c;
    sensor->angle_prev = 0;
    sensor->full_rotations = 0;
}

float AS5600_GetAngle(AS5600_t *sensor) {
    uint8_t data[2];
    HAL_StatusTypeDef status;
    // 增加超时判断，防止卡死
    // 0x36 << 1 = 0x6C 是写地址, 读取时 HAL 库会自动处理读写位
    status = HAL_I2C_Mem_Read(sensor->hi2c, 0x36 << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, data, 2, 10);
    if (status != HAL_OK) {
        return -1.0f; //返回-1表示错误
    }
    uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
    return (float)raw_angle * (2.0f * 3.14159265f) / 4096.0f;
}
