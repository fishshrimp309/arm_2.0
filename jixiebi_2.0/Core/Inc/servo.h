#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "tim.h"
#include "math.h"
#include "pca9685.h"
#include <stdbool.h>

#define angle_zero 500.0f
#define angle_rate 11.11111f
// 硬件参数（单位：mm）
#define L1 56.0f  // 底座底面到j1的高度
#define L2 40.0f  // j1-j2长度
#define L3 40.0f  // j2-j3长度
#define L4 89.0f  // J3到夹爪末端长度
#define M_PI 3.14159265358979323846f
//修正值
#define bias_0 0.0f
#define bias_1 0.0f
#define bias_2 0.0f
#define bias_3 0.0f 
#define bias_4 0.0f
#define bias_5 0.0f
#define bias_base 10.0f

typedef struct {
    float j[6];//
} IK_Result_t;
typedef struct {
    float x, y, z;
} FK_Result_t;

void servo_init(void);
IK_Result_t IK_Solve_Geometry(float x, float y, float z);
uint16_t servo_pwm_calculate(float angle);
void servo_xyz(float x, float y, float z, IK_Mode mode);
IK_Result_t IK_Solve_Core(float x, float y, float z, float pitch_deg);
IK_Result_t IK_Get_Target_Angle(float x, float y, float z,  IK_Mode mode);
float Constrain_Angle(float angle);
bool Check_Angle_Valid(IK_Result_t r);
FK_Result_t *FK_Solve_Core(float j0, float j1, float j2, float j3);

#endif
