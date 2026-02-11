#include "servo.h"

IK_Result_t result;

void servo_init(void) {
    PCA9685_Init(50.0f);
    PCA9685_SetServoAngle(0, 90);
    PCA9685_SetServoAngle(1, 120);
    PCA9685_SetServoAngle(2, 90);
    PCA9685_SetServoAngle(3, 90); // ÊÖÍó
    PCA9685_SetServoAngle(4, 90); // Ðý×ª
    PCA9685_SetServoAngle(5, 1500); // ¼Ð×¦
}

uint16_t servo_pwm_calculate(float angle)
{
    return (uint16_t)(angle_zero + angle_rate * angle);
}

void servo_xyz(float x, float y, float z, IK_Mode mode) {
    result = IK_Get_Target_Angle(x, y, z, mode);
	PCA9685_SetServoAngle(0, result.j[0] + bias_0); 
	PCA9685_SetServoAngle(1, result.j[1] + bias_1); 
	PCA9685_SetServoAngle(2, result.j[2] + bias_2); 
	PCA9685_SetServoAngle(3, result.j[3] + bias_3); 
}

IK_Result_t IK_Solve_Core(float x, float y, float z, float pitch_deg) {
    result.j[0] =90.0f - atan2(y, x) * (180.0f / M_PI);

    float pitch_rad = pitch_deg * (M_PI / 180.0f);
    float r_target = sqrt(x*x + y*y);
    float r_wrist = r_target - L4 * cos(pitch_rad);
    float z_wrist = (z - L1) - L4 * sin(pitch_rad);
    float vector_len_sq = r_wrist * r_wrist + z_wrist * z_wrist;

    float vector_len = sqrt(vector_len_sq);
    if (vector_len > (L2 + L3)) {
        vector_len = L2 + L3; 
    }

    float cos_angle_j3 = (L2*L2 + L3*L3 - vector_len_sq) / (2 * L2 * L3);
    if (cos_angle_j3 > 1.0f) cos_angle_j3 = 1.0f;
    if (cos_angle_j3 < -1.0f) cos_angle_j3 = -1.0f;
    float j3_inner_angle = acos(cos_angle_j3);
    float j3_math = j3_inner_angle * (180.0f / M_PI);
    result.j[2] =180.0f - (180.0f - j3_math + 90.0f);

    float j2_base_angle = atan2(z_wrist, r_wrist);
    float cos_angle_j2_offset = (L2*L2 + vector_len_sq - L3*L3) / (2 * L2 * vector_len);
    if (cos_angle_j2_offset > 1.0f) cos_angle_j2_offset = 1.0f;
    float j2_offset_angle = acos(cos_angle_j2_offset);
    float j2_math = (j2_base_angle + j2_offset_angle) * (180.0f / M_PI);
    result.j[1] =180.0f - j2_math; 

    result.j[3] =360.0f - (pitch_deg + result.j[1] + result.j[2]);

    return result;
}

    float target_pitch = 0.0f;
IK_Result_t IK_Get_Target_Angle(float x, float y, float z,  IK_Mode mode) {

    if (mode == MODE_AUTO_REACH) {
        float r = sqrt(x*x + y*y);
        float h = z - L1;
        target_pitch = atan2(h, r) * (180.0f / M_PI);
    }
     else if (mode == MODE_GRAB_FLAT) {
        target_pitch = 0.0f;
    }
     else if (mode == MODE_GRAB_DOWN) {
        target_pitch = -90.0f; 
    }
 
    return IK_Solve_Core(x, y, z, target_pitch);
}

