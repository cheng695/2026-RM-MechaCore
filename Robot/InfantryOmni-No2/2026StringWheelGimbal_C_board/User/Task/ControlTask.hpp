#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "core/Alg/Filter/Filter.hpp"
#include "core/Alg/Feedforward/Feedforward.hpp"
#include "CommunicationTask.hpp"
#include "core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.hpp"
#include "core/BSP/Common/FiniteStateMachine/FiniteStateMachine_launch.hpp"
#include "core/Alg/PID/pid.hpp"
#include "core/BSP/Motor/Dji/DjiMotor.hpp"
#include "core/BSP/Motor/DM/DmMotor.hpp"
#include "SerialTask.hpp"
#include "core/APP/Heat_Detector/Heat_Control_Private.hpp"

extern bool shoot;

typedef struct 
{
    float target_yaw;   
    float target_pitch;
    float target_dial;
    float target_surgewheel[2];
}ControlTask;

typedef struct
{
    float out_yaw_angle;
    float out_pitch_angle;
    float out_yaw;
    float out_pitch;
    bool  motor_pitch_enable; // J4310 Pitch 使能, ControlTask 置位, MotorTask 执行
    bool  motor_yaw_enable;   // J4310 Yaw   使能, ControlTask 置位, MotorTask 执行
}Output_gimbal;

typedef struct
{
    float out_dial;
    float out_surgewheel[2];
}Output_launch;

extern BSP::Motor::Dji::GM3508<3> Motor3508;
extern BSP::Motor::DM::J4310<2> MotorJ4310;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BoardCommunication Cboard;
extern Vision vision;

extern Output_gimbal gimbal_output;
extern Output_launch launch_output;
extern ControlTask gimbal_target;

extern Launch_FSM launch_fsm;
extern Gimbal_FSM gimbal_fsm;

extern APP::Heat_Control_Private heat_control;
extern Alg::Feedforward::GimbalFullCompensation gimbal_yaw;

extern Alg::Feedforward::UDE ude_yaw;

#endif
