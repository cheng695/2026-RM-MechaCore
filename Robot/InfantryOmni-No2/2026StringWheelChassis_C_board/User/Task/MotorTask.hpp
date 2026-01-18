#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "../User/core/HAL/CAN/can_hal.hpp"
#include "../User/Task/ControlTask.hpp"


typedef struct 
{
    float pm_voltage;
    float pm_current;
    float pm_power;
} powerMeter;
extern powerMeter pm01;


extern BSP::Motor::Dji::GM3508<4> Motor3508;
extern BSP::Motor::Dji::GM6020<4> Motor6020;


#endif
