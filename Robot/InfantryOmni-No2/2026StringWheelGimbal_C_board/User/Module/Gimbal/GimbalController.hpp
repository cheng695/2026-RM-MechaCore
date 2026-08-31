#pragma once

/*
 * 云台控制器入口。
 *
 * 根据云台状态机当前状态，选择对应的控制模式并生成电机控制输出。
 */
void RunGimbalController();

/* 设备在线状态由控制任务提供，云台规划在生成目标时读取该状态。 */
bool check_online();
