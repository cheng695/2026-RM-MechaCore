#pragma once

#include "ChassisStateMachine.hpp"
#include "ChassisTypes.hpp"
#include "CommunicationTask.hpp"
#include "DT7.hpp"
#include "DjiMotor.hpp"
#include "OmniCalculation.hpp"
#include "OperatorInput.hpp"
#include "PowerControl.hpp"

/*
 * 跨任务共享对象集中声明。
 *
 * Task 层只负责调度，不再通过互相 include 对方的 Task 头文件来借用全局对象。
 */
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BSP::Motor::Dji::GM3508<4> Motor3508;

extern BoardCommunication Cboard;
extern Supercapacitor supercap;

extern OperatorInput operator_input;

extern Chassis_FSM chassis_fsm;
extern ChassisTarget chassis_target;
extern ChassisOutput chassis_output;

/*
 * 底盘全向轮几何参数。
 *
 * IK 和 FK 必须使用同一组轮子安装参数，否则目标解算和速度反馈会出现模型不一致。
 */
extern float chassis_wheel_azimuth[4];
extern float chassis_wheel_direction[4];

/*
 * 当前底盘速度反馈会被控制和调试发送共同使用，因此放在共享上下文。
 */
extern Alg::CalculationBase::Omni_FK omni_fk;

/*
 * 能量环状态需要跨功率控制和调试观察时共享。
 */
extern ALG::PowerControl::EnergyRing energy_ring;
