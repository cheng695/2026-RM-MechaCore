#ifndef MOTORTASK_H
#define MOTORTASK_H

#include "core/HAL/CAN/can_hal.hpp"
#include "ControlTask.hpp"
  
extern ALG::PID::PID yaw_angle_pid;       
extern ALG::PID::PID yaw_velocity_pid;      
extern ALG::PID::PID pitch_angle_pid;        
extern ALG::PID::PID pitch_velocity_pid;
extern ALG::PID::PID dial_pid;
extern ALG::PID::PID surgewheel_pid[2];

extern Gimbal_FSM gimbal_fsm;


/* 达妙电机反馈缓存，供云台控制任务读取。 */
extern float Motor4310_angle[2];
extern float Motor4310_velocity[2];

#endif
