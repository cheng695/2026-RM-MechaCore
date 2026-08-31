#pragma once

#include <cstdint>

/* 云台专用输入，不包含发射机构的模式和动作字段。 */
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

    /* 规划层需要的当前姿态和操作量。 */
    float current_yaw = 0.0f;
    float current_J1 = 0.0f;
    float current_J2 = 0.0f;
    float mouse_x = 0.0f;
    float mouse_y = 0.0f;
    float right_x = 0.0f;
    float right_y = 0.0f;
    float vision_yaw = 0.0f;
    float vision_pitch = 0.0f;
};
