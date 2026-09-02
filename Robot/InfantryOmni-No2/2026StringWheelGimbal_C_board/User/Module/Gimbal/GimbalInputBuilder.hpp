#pragma once

#include "../OperatorInput.hpp"
#include "GimbalInput.hpp"

/* 从公共操作员输入中提取云台专用输入，不直接读取硬件。 */
GimbalInput BuildGimbalInput(const OperatorInput& operator_input);

/*
 * 构建云台控制器运行时输入。
 *
 * 该函数会额外读取 IMU、电机反馈、遥控器鼠标/摇杆和视觉数据，
 * 适合控制循环直接调用。
 */
GimbalInput BuildRuntimeGimbalInput();
