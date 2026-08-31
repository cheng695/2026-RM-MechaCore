#pragma once

#include <cstdint>

/* 发射机构专用输入，只保留发射状态机和控制层需要的数据。 */
struct LaunchInput
{
    uint8_t left_switch = 2;
    uint8_t right_switch = 2;

    bool equipment_online = false;
    bool keyboard_mode = false;
    bool vision_enabled = false;
    bool launch_enabled = false;
    bool mouse_left = false;
    bool mouse_left_pressed = false;
    bool mouse_left_released = false;

    /* 当前发射模式请求。单发动作本身不通过这两个字段触发。 */
    bool fire_single = false;
    bool fire_continuous = false;

    /* 卡弹检测结果，作为进入 JAM 的安全事件来源。 */
    bool jam_detected = false;

    /* 规划层需要的拨盘反馈、射速和鼠标滚轮输入。 */
    float current_dial_angle = 0.0f;
    float fire_rate = 0.0f;
    float scroll = 0.0f;
};
