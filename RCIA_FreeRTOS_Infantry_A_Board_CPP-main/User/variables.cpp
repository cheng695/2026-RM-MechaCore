#include "variables.hpp"

// const uint16_t master_id = 0x2;

DjiMotor motor_101(0x101);  //摩擦轮
DjiMotor motor_102(0x102);  //摩擦轮
DjiMotor motor_104(0x104);  //拨盘
DjiMotor motor_205(0x105);  //pitch
DjiMotor motor_206(0x206);  //yaw
DjiMotor motor_201(0x201);  //摩擦轮
DjiMotor motor_202(0x202);  //摩擦轮
DjiMotor motor_204(0x204);  //拨盘
DjiMotor* dji_motor_list[kMotorCount] = {
    &motor_202, &motor_204, &motor_206,&motor_201
};  //电机数组

//微分跟踪器 用于电机速度滤波
TD td_201(100, 0.001);
TD td_202(100, 0.001);
TD td_204(200, 0.001);
TD td_205(200, 0.001);
TD td_206(200, 0.001);
TD td_ch110_Z(800, 0.001);
PID pid_vel_201(12, 0.18, 0, 2500, 16384, 0);              //摩擦轮
PID pid_vel_202(12, 0.18, 0, 2500, 16384, 0);              //摩擦轮
//PID pid_vel_201(13, 0.18, 0, 2500, 16384, 0);              //摩擦轮
//PID pid_vel_202(13, 0.18, 0, 2500, 16384, 0);              //摩擦轮

PID pid_pos_204(0.12, 0, 0, 0, 10000, 0);                		//拨盘外环
PID pid_vel_204(4, 0.1, 0, 5000, 10000, 0);                     //拨盘内环

PID pid_pos_205(0.08, 0, 0, 20, 40, kPitchFeedForward);  //pitch外环
PID pid_vel_205(0.5, 0, 0, 0, 15, 0);                    //pitc h内环

PID pid_pos_206(110, 0, 150, 0, 5000, kYawFeedForward);      //yaw外环


PID pid_vel_206(15, 0.1, 30, 0, 16384 , 0);              //yaw内环

DR16 dr16;                               //遥控器接收机
CH110 ch110;                             //IMU
StateMachine state_machine;              //状态机
Vision vision;                           //视觉
EmpiricalGravityCompensator EGC(0);  //重力补偿
Communicator comm;                       //板间通信
RotateFeedForward RFF(1.7);              //小陀螺时YAW轴前馈
ErrorHandle error_handle;                //错误处理