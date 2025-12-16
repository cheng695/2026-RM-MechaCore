#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "../user/core/HAL/CAN/can_hal.hpp"
#include "../user/Task/ControlTask.hpp"

extern BSP::Motor::Dji::GM3508<4> Motor3508;
extern BSP::Motor::Dji::GM6020<1> Motor6020;
extern BSP::Motor::Dji::GM2006<1> Motor2006;
extern BSP::Motor::DM::J4310<1> MotorJ4310;

extern ALG::ADRC::FirstLADRC yaw_ladrc;
extern ALG::PID::PID yaw_angle_pid;
extern ALG::PID::PID yaw_velocity_pid;
extern ALG::PID::PID pitch_angle_pid;
extern ALG::PID::PID pitch_velocity_pid;
extern ALG::PID::PID dial_pid;
extern ALG::PID::PID surgewheel_pid[2];

#endif
