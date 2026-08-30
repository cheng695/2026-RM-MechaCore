#pragma once

#include "ChassisStateMachine.hpp"
#include "ChassisTypes.hpp"
#include "OperatorInput.hpp"

/*
 * 初始化底盘目标规划器。
 */
void InitChassisPlanner();

/*
 * 根据当前状态和操作员输入规划底盘目标速度。
 */
ChassisTarget PlanChassisTarget(Enum_Chassis_States state, const OperatorInput& input);
