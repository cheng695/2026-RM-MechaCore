#pragma once

/* 发射机构规划层输出的目标。 */
struct LaunchTarget
{
    float target_dial = 0.0f;
    float target_surgewheel[2] = {0.0f, 0.0f};
};

/* 发射机构控制器输出给电机任务的目标。 */
struct Output_launch
{
    float out_dial = 0.0f;
    float out_surgewheel[2] = {0.0f, 0.0f};
};
