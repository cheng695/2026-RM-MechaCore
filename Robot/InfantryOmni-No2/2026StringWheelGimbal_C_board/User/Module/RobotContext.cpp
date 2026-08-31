#include "RobotContext.hpp"
#include "Gimbal/GimbalTypes.hpp"

/* 机器人级共享对象集中定义，避免多个任务分别维护同一份状态。 */
Gimbal_FSM gimbal_fsm;
Launch_FSM launch_fsm;

GimbalTarget gimbal_planner_target;
LaunchTarget launch_planner_target;

Output_gimbal gimbal_output;
Output_launch launch_output;

Mit_K gimbal_k;
