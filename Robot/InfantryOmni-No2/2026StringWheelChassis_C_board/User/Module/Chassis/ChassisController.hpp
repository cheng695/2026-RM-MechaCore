#pragma once

#include "OperatorInput.hpp"

/*
 * 初始化底盘控制模块。
 */
void InitChassisControl();

/*
 * 底盘周期控制入口。
 */
void ChassisControlLoop(bool is_online, const OperatorInput& input);
