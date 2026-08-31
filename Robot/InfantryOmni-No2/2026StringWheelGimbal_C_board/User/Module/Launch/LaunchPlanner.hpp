#pragma once

#include "LaunchInput.hpp"
#include "LaunchStateMachine.hpp"
#include "LaunchTypes.hpp"

/* 初始化发射规划器。 */
void InitLaunchPlanner();

/* 根据输入和状态生成发射目标；当前阶段保留现有控制算法。 */
LaunchTarget PlanLaunchTarget(Enum_Launch_States state,
                              const LaunchInput& input,
                              const LaunchTarget& current_target);
