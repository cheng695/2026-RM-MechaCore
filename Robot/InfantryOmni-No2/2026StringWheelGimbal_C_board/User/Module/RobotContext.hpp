#pragma once

#include "Module/Gimbal/GimbalStateMachine.hpp"
#include "Module/Gimbal/GimbalTypes.hpp"
#include "Module/Control/LegacyControlTypes.hpp"
#include "Module/Launch/LaunchStateMachine.hpp"
#include "Module/Launch/LaunchTypes.hpp"

/* 机器人级共享控制上下文。各任务通过此处访问模块状态和规划结果。 */
extern Gimbal_FSM gimbal_fsm;
extern Launch_FSM launch_fsm;

/* 旧控制层综合目标，迁移期间保留。 */
extern ControlTask gimbal_target;

/* 新规划层目标。 */
extern GimbalTarget gimbal_planner_target;
extern LaunchTarget launch_planner_target;

/* 控制器输出，由电机任务消费。 */
extern Output_gimbal gimbal_output;
extern Output_launch launch_output;

extern Mit_K gimbal_k;
