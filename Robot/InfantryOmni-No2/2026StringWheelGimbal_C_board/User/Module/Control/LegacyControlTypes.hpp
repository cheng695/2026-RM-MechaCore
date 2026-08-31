#pragma once

/*
 * 旧控制任务使用的综合目标结构。
 * 仅用于兼容尚未迁移的 ControlTask，迁移完成后应删除。
 */
struct ControlTask
{
    float target_yaw = 0.0f;
    float target_pitch = 0.0f;
    float target_J1 = 0.0f;
    float target_J2 = 0.0f;
    float target_dial = 0.0f;
    float target_surgewheel[2] = {0.0f, 0.0f};
};
