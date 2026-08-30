#pragma once

/*
 * 底盘目标速度。
 *
 * target_translation_x / target_translation_y 是底盘坐标系下的平移目标；
 * target_rotation 是底盘自转角速度目标。
 */
struct ChassisTarget
{
    float target_translation_x = 0.0f;
    float target_translation_y = 0.0f;
    float target_rotation = 0.0f;
};

/*
 * 底盘最终输出。
 *
 * out_wheel 存放四个 3508 轮电机的电流指令，单位为 DJI 电调控制值。
 */
struct ChassisOutput
{
    float out_wheel[4] = {0.0f, 0.0f, 0.0f, 0.0f};
};
