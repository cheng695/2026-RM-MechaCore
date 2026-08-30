#pragma once

#include <stdint.h>

/*
 * 操作员输入快照。
 *
 * 该结构体用于替代旧的 alphabet[28] 魔法数组，把遥控器、键鼠、开关和组合键
 * 统一封装成有语义的字段。SerialTask 负责更新该结构体，ControlTask 只读取
 * 这里的结果，不再关心具体按键索引。
 */
struct OperatorInput
{
    /* 平移输入，范围通常为 -1.0f 到 1.0f。 */
    float move_x = 0.0f;
    float move_y = 0.0f;

    /* 旋转输入，键鼠模式下由小陀螺键给出，遥控器模式下由拨轮给出。 */
    float rotate = 0.0f;

    /* 当前是否处于键鼠模式。 */
    bool keyboard_mode = false;

    /* 小陀螺输入，当前由 X 键按下状态生成。 */
    bool spin = false;

    /* 加速/超电输入，当前由 Shift 按下状态生成。 */
    bool boost = false;

    /* 主动补电模式，当前由 Ctrl 上升沿切换，并会在加速或开火时退出。 */
    bool charge = false;

    /* 底盘是否跟随云台，当前由 Q 键 toggle 状态生成。 */
    bool follow_enabled = true;

    /* 手动强制不信任裁判系统数据，当前由 Ctrl + Shift + F 触发。 */
    bool force_referee_untrusted = false;

    /* 手动恢复信任裁判系统数据，当前由 Ctrl + Shift + R 触发。 */
    bool force_referee_trusted = false;

    /* 遥控器左右三挡开关状态，默认 2 表示下挡。 */
    uint8_t left_switch = 2;
    uint8_t right_switch = 2;
};
