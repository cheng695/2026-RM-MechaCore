#pragma once

#include <cstdint>

/*
 * 串口任务整理后的公共操作员输入快照。
 *
 * 这里保存遥控器、键盘和鼠标产生的基础输入，以及当前阶段仍在共用的
 * 视觉和发射模式结果。后续可由 GimbalInputBuilder 和 LaunchInputBuilder
 * 分别提取，最终移除不属于公共层的字段。
 */
struct OperatorInput
{
    uint8_t left_switch = 2;
    uint8_t right_switch = 2;

    bool keyboard_mode = false;
    bool mouse_left = false;
    bool mouse_left_pressed = false;
    bool mouse_right = false;
    bool key_x = false;
    bool key_z = false;
    bool key_v = false;
    bool key_b = false;
    bool key_r = false;
    bool key_g = false;

    bool vision_enabled = false;
    bool transform_requested = false;
    bool launch_enabled = false;
    bool fire_single = false;
    bool fire_continuous = false;
    bool mouse_left_released = false;
};
