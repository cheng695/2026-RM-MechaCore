#pragma once

#include "GimbalInput.hpp"

/*
 * 云台控制器入口。
 *
 * 该模块负责：
 * 1. 读取 OperatorInput、视觉数据和电机反馈，构建 GimbalInput；
 * 2. 调用 GimbalEventDetector 生成状态机事件；
 * 3. 调用 GimbalPlanner 生成 yaw/J1/J2 目标；
 * 4. 将目标转换为 MotorTask 可直接发送的达妙 MIT 控制输出。
 */
void RunGimbalController();

/* 初始化云台控制模块，包括状态机和规划器。 */
void InitGimbalControl();

/* 云台控制主循环，调用者提供在线状态和本周期云台输入。 */
void GimbalControlLoop(bool is_online, const GimbalInput& input);

/*
 * 设备在线状态由 ControlTask 提供。
 * 云台控制器只读取该结果，不直接决定哪些设备算在线。
 */
bool check_online();
