#ifndef CONTROLTASK_HPP
#define CONTROLTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/Alg/Filter/Filter.hpp"
#include "../User/core/Alg/Feedforward/Feedforward.hpp"
#include "../User/Task/CommunicationTask.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_gimbal.hpp"
#include "../User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_launch.hpp"
#include "../User/core/Alg/PID/pid.hpp"
#include "../User/core/Alg/ADRC/adrc.hpp"
#include "../User/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../User/core/BSP/Motor/DM/DmMotor.hpp"
#include "../User/core/BSP/Motor/LK/Lk_motor.hpp"
#include "../User/Task/SerialTask.hpp"
#include "../User/core/APP/Heat_Detector/Heat_Control_Private.hpp"
#include "../User/core/Alg/VMC/VMC.hpp"
#include "../User/core/Alg/UtilityFunction/SlopePlanning.hpp"

extern bool shoot;

typedef struct 
{
    float target_yaw_vel;       // 速度闭环
    float target_yaw_angle;     // 角度设置（transform用）
    float target_pitch1;        //  VMC控制�?
    float target_pitch2_coder;  //  编码器控制用
    float target_pitch2_vel;    //  速度闭环控制
    float target_pitch2_angle;  //  角度设置（transform以及VMC用）  
    
    float target_dial;
    float target_surgewheel[2];
}ControlTask;

typedef struct
{
    float out_yaw;
    float out_pitch1;
    float out_pitch2;
}Output_gimbal;

typedef struct
{
    float out_dial;
    float out_surgewheel[2];
}Output_launch;

extern BSP::Motor::Dji::GM3508<3> Motor3508;
extern BSP::Motor::DM::J4310<2> MotorJ4310;
extern BSP::Motor::DM::J4340<1> MotorJ4340;
// extern BSP::Motor::LK::LK4005<1> MotorLK4005;

extern BSP::IMU::HI12_float HI12;
extern BSP::REMOTE_CONTROL::RemoteController DT7;
extern BoardCommunication Cboard;
extern Vision vision;

extern Output_gimbal gimbal_output;
extern Output_launch launch_output;
extern ControlTask gimbal_target;

extern Launch_FSM launch_fsm;
extern Gimbal_FSM gimbal_fsm;

extern APP::Heat_Control_Private heat_control;

extern ALG::ADRC::FirstLADRC yaw_adrc;
extern ALG::ADRC::FirstLADRC pitch2_adrc;
extern ALG::PID::PID yaw_transform_angle_pid;
extern ALG::ADRC::FirstLADRC yaw_adrc_try;
extern Alg::Feedforward::GimbalFullCompensation gimbal_yaw;

#endif
