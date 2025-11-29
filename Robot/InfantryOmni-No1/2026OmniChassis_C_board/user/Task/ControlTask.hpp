#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/Alg/ChassisCalculation/OmniCalculation.hpp"
#include "../user/core/Alg/PID/pid.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"

extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BSP::Motor::Dji::GM3508<4> Motor3508;

#endif
