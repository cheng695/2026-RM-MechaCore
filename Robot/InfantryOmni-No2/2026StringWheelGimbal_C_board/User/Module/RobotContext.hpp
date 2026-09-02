#pragma once

#include "Module/Gimbal/GimbalStateMachine.hpp"
#include "Module/Gimbal/GimbalTypes.hpp"
#include "Module/Launch/LaunchStateMachine.hpp"
#include "Module/Launch/LaunchTypes.hpp"

/* 机器人级共享控制上下文。各任务通过这里访问模块状态和规划结果。 */
extern Gimbal_FSM gimbal_fsm;
extern Launch_FSM launch_fsm;

/* 规划层输出的目标值。 */
extern GimbalTarget gimbal_planner_target;
extern LaunchTarget launch_planner_target;

/* 控制器输出，最终由电机任务转换成电机控制帧。 */
extern Output_gimbal gimbal_output;
extern Output_launch launch_output;

extern Mit_K gimbal_k;
