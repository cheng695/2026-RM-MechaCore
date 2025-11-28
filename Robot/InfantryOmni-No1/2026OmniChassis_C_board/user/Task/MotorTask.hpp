#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/HAL/CAN/can_hal.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"

extern BSP::Motor::Dji::GM3508<4> Motor3508;

#endif
