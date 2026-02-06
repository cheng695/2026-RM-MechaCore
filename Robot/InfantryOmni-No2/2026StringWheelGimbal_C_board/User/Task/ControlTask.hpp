#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/Alg/Filter/Filter.hpp"
#include "../User/core/Alg/Feedforward/Feedforward.hpp"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_launch.hpp"
#include "../User/core/Alg/PID/pid.hpp"
#include "../User/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../User/core/BSP/Motor/DM/DmMotor.hpp"
#include "../User/Task/SerialTask.hpp"
#include "../User/core/APP/Heat_Detector/Heat_Control_Private.hpp"

extern bool shoot;

typedef struct 
{
    float target_yaw;   
    float target_pitch;
    float target_pitch_vel; // Combined logic refactor
    float target_dial;
    float target_surgewheel[2];
}ControlTask;

typedef struct
{
    float out_yaw;
    float out_pitch;
}Output_gimbal;

typedef struct
{
    float out_dial;
    float out_surgewheel[2];
}Output_launch;

extern BSP::Motor::Dji::GM3508<3> Motor3508;
extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern BSP::Motor::Dji::GM2006<1> Motor2006;
extern BSP::Motor::DM::J4310<1> MotorJ4310;

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

#endif
