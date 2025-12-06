#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.hpp"
#include "../user/core/Alg/PID/pid.hpp"
#include "../user/core/Alg/ADRC/adrc.hpp"
#include "../user/core/BSP/IMU/HI12_imu.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern BSP::Motor::Dji::GM3508<2> Motor3508;
extern BSP::Motor::Dji::GM2006<1> Motor2006;
extern BSP::Motor::DM::J4310<1> MotorJ4310;

// extern void chassis_stop();
// extern void chassis_not_follow();

#endif
