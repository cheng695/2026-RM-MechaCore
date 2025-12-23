#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/core/BSP/IMU/HI12_imu.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.hpp"
#include "../user/core/Alg/ChassisCalculation/StringWheel.hpp"
#include "../User/core/Alg/PID/pid.hpp"
#include "../User/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../User/core/BSP/Motor/LK/Lk_motor.hpp"

typedef struct 
{
    float target_translation_x;
    float target_translation_y;
    float target_rotation;
}ControlTask;

typedef struct
{
    float out_string[4];
    float out_wheel[4];
}Output_chassis;


extern BSP::Motor::Dji::GM3508<4> Motor3508;
extern BSP::Motor::Dji::GM6020<4> Motor6020;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BoardCommunication Cboard;

extern Output_chassis chassis_output;
extern float motor_direction[4];

#endif
