#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "../User/Task/ControlTask.hpp"
#include "Lk_motor.hpp"
#include "../User/core/BSP/Common/CANTransport/CANTransport.hpp"

extern BSP::Motor::Dji::GM3508<4> Motor3508;
extern BSP::Motor::LK::LK4005<4> Motor4005;
extern BSP::CANTransport::RxBuffer board_downlink_rx;

#endif
