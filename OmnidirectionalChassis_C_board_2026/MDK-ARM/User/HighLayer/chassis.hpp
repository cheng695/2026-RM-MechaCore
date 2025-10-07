#ifndef _CHASSIS_HPP_
#define _CHASSIS_HPP_ 

#include "FreeRTOS.h"
#include <cmsis_os.h>
#include "../User/HighLayer/Monitor.hpp"
#include "../User/HighLayer/Control.hpp"

extern Monitor::motor_monitor MOTOR_MONITOR;
extern Monitor::remote_monitor REMOTE_MONITOR;
extern Control::motor_control wheel_control;
extern Control::remote_control remote_control;

#endif
