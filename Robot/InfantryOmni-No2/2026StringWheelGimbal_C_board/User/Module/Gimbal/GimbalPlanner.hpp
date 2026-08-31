#pragma once

#include "GimbalInput.hpp"
#include "GimbalStateMachine.hpp"
#include "GimbalTypes.hpp"

/* 初始化云台规划器。 */
void InitGimbalPlanner();

/* 根据输入和状态生成云台目标；当前阶段保留现有控制算法。 */
GimbalTarget PlanGimbalTarget(Enum_Gimbal_States state,
                              GimbalTransition transition,
                              const GimbalInput& input,
                              const GimbalTarget& current_target);
