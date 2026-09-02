#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "core/Alg/Filter/Filter.hpp"
#include "core/Alg/Feedforward/Feedforward.hpp"
#include "CommunicationTask.hpp"
#include "Module/Gimbal/GimbalStateMachine.hpp"
#include "Module/Gimbal/GimbalTypes.hpp"
#include "Module/Launch/LaunchStateMachine.hpp"
#include "Module/Launch/LaunchTypes.hpp"
#include "core/Alg/PID/pid.hpp"
#include "core/BSP/Motor/Dji/DjiMotor.hpp"
#include "core/BSP/Motor/DM/DmMotor.hpp"
#include "core/BSP/Motor/LK/Lk_motor.hpp"
#include "SerialTask.hpp"
#include "core/APP/Heat_Detector/Heat_Control_Private.hpp"

extern bool shoot;

extern BSP::Motor::Dji::GM3508<2> Motor3508;
extern BSP::Motor::DM::J4310<2> MotorJ4310;
extern BSP::Motor::DM::J4340<1> MotorJ4340;
extern BSP::Motor::LK::LK4005<1> MotorLK4005;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BoardCommunication Cboard;
extern Vision vision;

extern Output_gimbal gimbal_output;
extern Output_launch launch_output;
extern Launch_FSM launch_fsm;
extern Gimbal_FSM gimbal_fsm;

extern APP::Heat_Control_Private heat_control;
extern Alg::Feedforward::GimbalFullCompensation gimbal_yaw;
extern Alg::Feedforward::UDE ude_yaw;

#endif
