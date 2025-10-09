#ifndef _CHASSIS_HPP_
#define _CHASSIS_HPP_ 

#include "FreeRTOS.h"
#include <cmsis_os.h>
#include "../User/HighLayer/Monitor.hpp"
#include "../User/HighLayer/Control.hpp"
#include "../User/HighLayer/Communication.hpp"

extern Monitor::motor_monitor MOTOR_MONITOR;
extern Monitor::remote_monitor REMOTE_MONITOR;
extern Monitor::board_monitor BOARD_MONITOR;
extern Control::motor_control wheel_control;
extern Control::remote_control remote_control;
extern Communication::communication Comm;

#endif
