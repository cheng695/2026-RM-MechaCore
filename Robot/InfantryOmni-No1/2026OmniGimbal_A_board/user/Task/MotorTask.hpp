#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/HAL/CAN/can_hal.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../user/core/BSP/Motor/DM/DmMotor.hpp"
//#include "../user/core/BSP/Motor/LK/Lk_motor.hpp"
#include "../user/core/Alg/PID/pid.hpp"
#include "../user/core/Alg/ADRC/adrc.hpp"
#include "../user/core/HAL/LOGGER/logger.hpp"

extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern BSP::Motor::Dji::GM3508<2> Motor3508;
extern BSP::Motor::Dji::GM2006<1> Motor2006;
extern BSP::Motor::DM::J4310<1> MotorJ4310;
extern ALG::ADRC::FirstLADRC yaw_adrc;
extern ALG::PID::PID pitch_AnglePid;
extern ALG::PID::PID pitch_VelocityPid;
extern ALG::PID::PID yaw_pid;
extern ALG::PID::PID dial_pid;
extern ALG::PID::PID surgewheel_pid[2];


#endif
