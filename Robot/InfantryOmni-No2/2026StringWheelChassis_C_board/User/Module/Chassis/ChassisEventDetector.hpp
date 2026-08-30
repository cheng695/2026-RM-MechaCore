#pragma once

#include "OperatorInput.hpp"
#include "ChassisStateMachine.hpp"

/*
 * 将操作员输入和设备在线状态解释为底盘状态机事件。
 *
 * 该模块理解遥控器左右开关、Q 键跟随开关等输入语义；
 * FSM 本身只接收 StopRequested / FollowRequested / NotFollowRequested。
 */
ChassisEvent ResolveChassisEvent(const OperatorInput& input, bool equipment_online);
ChassisEvent DetectChassisEvent(const OperatorInput& input, bool equipment_online);
