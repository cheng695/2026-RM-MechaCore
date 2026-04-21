#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include <algorithm>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/core/BSP/IMU/HI12_imu.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.hpp"
#include "../user/core/Alg/ChassisCalculation/OmniCalculation.hpp"
#include "../User/core/Alg/PID/pid.hpp"
#include "../User/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../User/core/BSP/Motor/LK/Lk_motor.hpp"
#include "../User/core/Alg/UtilityFunction/SlopePlanning.hpp"
#include "../User/core/Alg/PowerControl/PowerControl.hpp"

typedef struct 
{
    float target_translation_x;
    float target_translation_y;
    float target_rotation;
    float target_dial;
}ControlTask;

typedef struct
{
    float out_wheel[4];
    float out_dial;
}Output_chassis;


extern BSP::Motor::Dji::GM3508<4> Motor3508;
extern BSP::Motor::LK::LK4005<1> MotorLK4005;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BoardCommunication Cboard;

extern ControlTask chassis_target;
extern Output_chassis chassis_output;
extern float motor_wheel[4];

extern Alg::CalculationBase::Omni_FK omni_fk;
extern Alg::CalculationBase::Omni_IK omni_ik;
extern Alg::Utility::SlopePlanning omni_target[3];

extern ALG::PowerControl::EnergyRing energy_ring;

#endif
