#include "servo.h"

IK_Result_t result = {
    .j = {M_PI_2, 90.0f, 90.0f, 90.0f, 90.0f, 160.0f},
    .ready = 0
};
FK_Result_t res[4];

void servo_init(void) {
    PCA9685_Init(50.0f);
    // PCA9685_SetServoAngle(0, 90);//腰
    PCA9685_SetServoAngle(1, 90);//肩
    PCA9685_SetServoAngle(2, 90);//肘
    PCA9685_SetServoAngle(3, 90);//腕上下
    PCA9685_SetServoAngle(4, 90);//腕旋转
    PCA9685_SetServoAngle(5, 160);//夹爪
}

uint16_t servo_pwm_calculate(float angle)
{
    return (uint16_t)(angle_zero + angle_rate * angle);
}

void servo_xyz(void) {
    // result = IK_Get_Target_Angle(x, y, z, mode);
//	PCA9685_SetServoAngle(0, result.j[0] + bias_0); 
	PCA9685_SetServoAngle(1, result.j[1] + bias_1); 
	PCA9685_SetServoAngle(2, result.j[2] + bias_2); 
	PCA9685_SetServoAngle(3, result.j[3] + bias_3); 
}

IK_Result_t IK_Solve_Core(float x, float y, float z, float pitch_deg) {
    bool is_negative_x = false;
    if(x < 0){
        is_negative_x = true;
        x = -x;
    }//镜像法模背面

    // result.j[0] =90.0f - atan2f(y, x) * (180.0f / M_PI);
    result.j[0] =M_PI_2 - atan2f(y, x);

    float pitch_rad = pitch_deg * (M_PI / 180.0f);
    float r_target = sqrtf(x*x + y*y) - bias_base;//j1不在底座中心，减去偏差值
    if(is_negative_x) r_target = sqrtf(x*x + y*y) + bias_base; //背面加偏差值
    float r_wrist = r_target - L4 * cosf(pitch_rad);
    float z_wrist = (z - L1) - L4 * sinf(pitch_rad);
    float vector_len_sq = r_wrist * r_wrist + z_wrist * z_wrist;

    float vector_len = sqrtf(vector_len_sq);
    if (vector_len > (L2 + L3)) {
        vector_len = L2 + L3; 
    }

    float cos_angle_j3 = (L2*L2 + L3*L3 - vector_len_sq) / (2 * L2 * L3);
    if (cos_angle_j3 > 1.0f) cos_angle_j3 = 1.0f;
    if (cos_angle_j3 < -1.0f) cos_angle_j3 = -1.0f;
    float j3_inner_angle = acosf(cos_angle_j3);
    float j3_math = j3_inner_angle * (180.0f / M_PI);
    // result.j[2] =180.0f - (180.0f - j3_math + 90.0f);
    result.j[2] =270.0f - j3_math;

    float j2_base_angle = atan2f(z_wrist, r_wrist);
    float cos_angle_j2_offset = (L2*L2 + vector_len_sq - L3*L3) / (2 * L2 * vector_len);
    if (cos_angle_j2_offset > 1.0f) cos_angle_j2_offset = 1.0f;
    float j2_offset_angle = acosf(cos_angle_j2_offset);
    float j2_math = (j2_base_angle + j2_offset_angle) * (180.0f / M_PI);
    result.j[1] =180.0f - j2_math; 

    result.j[3] =360.0f - (pitch_deg + result.j[1] + result.j[2]);

    if(is_negative_x){
//        result.j[0] = 180.0f - result.j[0];
		result.j[0] = M_PI - result.j[0];
        result.j[1] = 180.0f - result.j[1];
        result.j[2] = 180.0f - result.j[2];
        result.j[3] = 180.0f - result.j[3];
    }

    return result;
}

float p_log;  
IK_Result_t IK_Get_Target_Angle(float x, float y, float z,  IK_Mode mode) {
    float target_pitch = 0.0f; 
    if (mode == MODE_AUTO_REACH) {
        float r = sqrtf(x*x + y*y);
        float h = z - L1;
        target_pitch = atan2f(h, r) * (180.0f / M_PI);
    }
     else if (mode == MODE_GRAB_FLAT) {
        target_pitch = 0.0f;
    }
     else if (mode == MODE_GRAB_DOWN) {
        target_pitch = -90.0f; 
    }
 
    IK_Result_t res;

    for (int i = 0; i < 18; i++) {
        res = IK_Solve_Core(x, y, z, target_pitch + (i*5.0f));
        p_log = target_pitch + (i*5.0f);
        if (Check_Angle_Valid(res)) return res;
        res = IK_Solve_Core(x, y, z, target_pitch - (i*5.0f));
        p_log = target_pitch - (i*5.0f);
        if (Check_Angle_Valid(res)) return res;
    }
     p_log = 100000.0f;
     return IK_Solve_Core(x, y, z, target_pitch);
    // return IK_Solve_Core(x, y, z, target_pitch);
}

bool Check_Angle_Valid(IK_Result_t r) {
    if (r.j[1] < 0 || r.j[1] > 180) return false;
    if (r.j[2] < 0 || r.j[2] > 180) return false;
    if (r.j[3] < 0 || r.j[3] > 180) return false;
    return true;
}


FK_Result_t *FK_Solve_Core(float j0, float j1, float j2, float j3) {
    
    float j1_rad = (180.0f - j1) * (M_PI / 180.0f);
    float j2_rad = (90.0f - (j1 - 90.0f + j2 - 90.0f)) * (M_PI / 180.0f);
    float j3_rad = (90.0f - (j1 - 90.0f + j2 - 90.0f + j3 - 90.0f)) * (M_PI / 180.0f);
    float x1 = bias_base;
    float z1 = L1;
    float x2 = x1 + L2 * cosf(j1_rad);
    float z2 = z1 + L2 * sinf(j1_rad);
    float x3 = x2 + L3 * cosf(j2_rad);
    float z3 = z2 + L3 * sinf(j2_rad);
    float x4 = x3 + L4 * cosf(j3_rad);
    float z4 = z3 + L4 * sinf(j3_rad);
    res[0].x = x1;
    res[0].z = z1;
    res[1].x = x2;
    res[1].z = z2;
    res[2].x = x3;
    res[2].z = z3;
    res[3].x = x4;
    res[3].z = z4;
    return res;
}
