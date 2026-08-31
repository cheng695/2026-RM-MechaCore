#pragma once

#include <cstdint>

/* 云台规划层输出的目标。 */
struct GimbalTarget
{
    float target_yaw = 0.0f;
    float target_J1 = 0.0f;
    float target_J2 = 0.0f;
};

/* 云台控制器输出给电机任务的目标。 */
struct Output_gimbal
{
    float out_yaw_torque = 0.0f;
    float out_yaw_angle = 0.0f;
    float out_yaw_velocity = 0.0f;

    float out_J1_torque = 0.0f;
    float out_J1_angle = 0.0f;
    float out_J1_velocity = 0.0f;

    float out_J2_torque = 0.0f;
    float out_J2_angle = 0.0f;
    float out_J2_velocity = 0.0f;
};

struct Mit_K
{
    float J1_Kp = 0.0f;
    float J1_Kd = 0.0f;

    float J2_Kp = 0.0f;
    float J2_Kd = 0.0f;

    float yaw_Kp = 0.0f;
    float yaw_Kd = 0.0f;
};