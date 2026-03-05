//#include "pid.h"

//void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float limit) {
//    pid->Kp = kp;
//    pid->Ki = ki;
//    pid->Kd = kd;
//    pid->output_limit = limit;
//    pid->integral = 0.0f;
//    pid->error_prev = 0.0f;
//}

//void PID_Reset(PID_Controller *pid) {
//    pid->integral = 0.0f;
//    pid->error_prev = 0.0f;
//}

//float PID_Compute(PID_Controller *pid, float target, float current) {
//    float error = target - current;
//    //角度最短路径处理
//    if (error > 3.14159265f) {
//        error -= 6.2831853f;
//    } else if (error < -3.14159265f) {
//        error += 6.2831853f;
//    }
//    pid->integral += error;
//    float max_integral = pid->output_limit / (pid->Ki + 0.0001f);
//    if (pid->Ki > 0.0f) {
//        if (pid->integral > max_integral) pid->integral = max_integral;
//        if (pid->integral < -max_integral) pid->integral = -max_integral;
//    }

//    float derivative = error - pid->error_prev;
//    pid->error_prev = error; // 更新误差

//    //计算总输出
//    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

//    //输出限幅
//    if (output > pid->output_limit) {
//        output = pid->output_limit;
//    } else if (output < -pid->output_limit) {
//        output = -pid->output_limit;
//    }

//    return output;
//}
