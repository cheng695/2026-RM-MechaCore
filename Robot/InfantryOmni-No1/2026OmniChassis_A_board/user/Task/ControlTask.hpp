#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/Task/CommunicationTask.hpp"
#include "../user/core/BSP/IMU/HI12_imu.hpp"
#include "../user/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.hpp"
#include "../user/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_launch.hpp"
#include "../user/core/Alg/PID/pid.hpp"
#include "../user/core/Alg/ADRC/adrc.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"

typedef struct 
{
    float target_yaw;   
    float target_pitch;
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

extern BSP::Motor::Dji::GM3508<4> Motor3508;
extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern BSP::Motor::Dji::GM2006<1> Motor2006;
extern BSP::Motor::DM::J4310<1> MotorJ4310;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BoardCommunication Aboard;

extern Output_gimbal gimbal_output;
extern Output_launch launch_output;

#endif
