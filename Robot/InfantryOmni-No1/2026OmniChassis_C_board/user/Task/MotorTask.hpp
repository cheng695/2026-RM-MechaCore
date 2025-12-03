#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/HAL/CAN/can_hal.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../user/core/Alg/PID/pid.hpp"

extern BSP::Motor::Dji::GM3508<4> Motor3508;
extern ALG::PID::PID wheel_pid[4];

#endif
