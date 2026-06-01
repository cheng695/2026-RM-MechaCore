#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "core/HAL/CAN/can_hal.hpp"
#include "core/BSP/Common/CANTransport/CANTransport.hpp"
#include "ControlTask.hpp"
  
extern ALG::PID::PID yaw_angle_pid;       
extern ALG::PID::PID yaw_velocity_pid;      
extern ALG::PID::PID pitch_angle_pid;        
extern ALG::PID::PID pitch_velocity_pid;
extern ALG::PID::PID dial_pid;
extern ALG::PID::PID surgewheel_pid[2];

extern Gimbal_FSM gimbal_fsm;

extern BSP::CANTransport::RxBuffer board_downlink_rx;

#endif
