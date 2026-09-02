#pragma once

#include <cstdint>

/*
 * 云台专用输入，不包含发射机构的模式和动作字段。
 *
 * 该结构体是 GimbalController 的输入快照：
 * 1. 开关、键鼠和视觉使能来自 OperatorInput；
 * 2. 当前姿态、电机角度、电机速度来自 GimbalInputBuilder；
 * 3. Controller 和 Planner 只读取这个结构体，不直接读遥控器、IMU 或电机对象。
 */
struct GimbalInput
{
    uint8_t left_switch = 2;
    uint8_t right_switch = 2;

    bool keyboard_mode = false;
    bool mouse_right = false;

    bool key_x = false;
    bool key_v = false;

    bool vision_enabled = false;
    bool transform_requested = false;
    bool chassis_spin = false;         /* 底盘小陀螺标志，用于决定 yaw 是否启用 UDE 前馈。 */

    /* 当前姿态和关节反馈。 */
    float current_yaw = 0.0f;          /* 叠加 yaw，单位 rad，用于手动模式连续转动。 */
    float current_yaw_angle = 0.0f;    /* 普通 yaw，单位 rad，用于视觉/变形的绝对角度闭环。 */
    float current_yaw_velocity = 0.0f; /* yaw 角速度，单位 rad/s，用于 yaw 前馈计算。 */
    float current_pitch = 0.0f;        /* IMU pitch，单位 rad，用于 J2 外环角度闭环。 */
    float current_J1 = 0.0f;           /* J1 电机反馈角度，单位 rad。 */
    float current_J2 = 0.0f;           /* J2 电机反馈角度，单位 rad。 */
    float current_J1_velocity = 0.0f;  /* J1 电机反馈角速度，单位 rad/s。 */
    float current_J2_velocity = 0.0f;  /* J2 电机反馈角速度，单位 rad/s。 */

    /* 操作输入和视觉输入。 */
    float mouse_x = 0.0f;
    float mouse_y = 0.0f;
    float right_x = 0.0f;
    float right_y = 0.0f;
    float vision_yaw = 0.0f;
    float vision_pitch = 0.0f;
};
