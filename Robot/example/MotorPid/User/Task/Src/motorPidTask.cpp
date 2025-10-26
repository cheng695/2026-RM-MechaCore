#include "User/Task/Inc/motorPidTask.hpp"
#include "User/config.h"
#include "cmsis_os2.h"

void MotorPidTask(void *argument)
{
    static TASK::MotorPid::MotorPid motorPid;

    for (;;)
    {
        motorPid.UpData();
        osDelay(MOTOR_CONTROL_HZ);
    }
}

namespace TASK::MotorPid
{
MotorPid::MotorPid()
    : // 位置pid
      pid_pos(0.0, 0.0, 0.0, 0.0),
      // 速度pid
      pid_vel(0.0, 0.0, 0.0, 0.0),
      // 速度adrc
      adrc_vel(ALG::LADRC::TDquadratic(100, 0.001), 0.0, 0.0, 0.0, 0.001f, 0.0)
{
    // 获取CAN设备实例
    auto &chassis_can = CAN_INSTANCE.get_device(CHASSIS_CAN);
    // 注册电机的Parse函数作为CAN接收回调
    Motor6020.registerCallback(&chassis_can);
}

void MotorPid::UpData()
{
    Status[Now_Status_Serial].Count_Time++; // 计时

    switch (Now_Status_Serial)
    {
    case (MotorPid_Status::DISABLE): {
        Disable();

        break;
    }
    case (MotorPid_Status::PID): {
        PidUpData();

        break;
    }
    case (MotorPid_Status::ADRC): {
        AdrcUpData();

        break;
    }
    }

    sendCan();
}

void MotorPid::Disable(void)
{
    // 读取当前状态作为目标
    target_pos = Motor6020.getAngleDeg(2);
    target_vel = Motor6020.getVelocityRads(2);

    // 目标跟随当前值，控制器输出为0
    // 位置环：目标位置 -> 期望速度
    pid_pos.setTarget(target_pos);
    pid_pos.UpData(Motor6020.getAngleDeg(2));

    // 速度环：期望速度 -> 控制量
    pid_vel.setTarget(pid_pos.getOutput());
    pid_vel.UpData(Motor6020.getVelocityRads(2));
}

void MotorPid::PidUpData()
{
    // 读取电机反馈
    float cur_pos = Motor6020.getAngleDeg(2);
    float cur_vel = Motor6020.getVelocityRads(2);

    // 位置环：目标位置 -> 期望速度
    pid_pos.setTarget(target_pos);
    pid_pos.UpData(cur_pos);

    // 速度环：期望速度 -> 控制量
    pid_vel.setTarget(pid_pos.getOutput());
    pid_vel.UpData(cur_vel);
}

void MotorPid::AdrcUpData()
{
    // 读取电机反馈
    float cur_pos = Motor6020.getAngleDeg(2);
    float cur_vel = Motor6020.getVelocityRads(2);

    // 位置环：目标位置 -> 期望速度
    pid_pos.setTarget(target_pos);
    pid_pos.UpData(cur_pos);

    // 速度环(ADRC)：期望速度 -> 控制量
    adrc_vel.setTarget(pid_pos.getOutput());
    adrc_vel.UpData(cur_vel);
}

void MotorPid::sendCan(void)
{
    // 获取地址
    auto &can_motor = CAN_INSTANCE.get_device(CHASSIS_CAN);

    // 发送数据
    Motor6020.sendCAN(&can_motor);
}
} // namespace TASK::MotorPid
