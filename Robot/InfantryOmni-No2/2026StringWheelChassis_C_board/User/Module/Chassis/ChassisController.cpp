#include "ChassisController.hpp"
#include "ChassisEventDetector.hpp"
#include "ChassisPlanner.hpp"
#include "RobotContext.hpp"
#include "OmniCalculation.hpp"
#include "PowerControl.hpp"
#include "RM_RefereeSystem.h"
#include "pid.hpp"

#include <algorithm>
#include <cmath>

namespace
{
/************************************* 参数定义 *************************************/
// 底盘逆运动学
Alg::CalculationBase::Omni_IK omni_ik(0.24f, 0.07f, chassis_wheel_azimuth, chassis_wheel_direction);

// 轮向电机 pid
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 150.0f)};

// 能量环 pid
ALG::PID::PID energy_pid[2] = {
    ALG::PID::PID(15.0f, 0.0f, 10.0f, 16384.0f, 0.0f, 0.0f),
    ALG::PID::PID(4.0f, 0.0f, 10.0f, 16384.0f, 0.0f, 0.0f)};

// 功率控制
ALG::PowerControl::PowerControl<4> power3508;   // 功率分配
ALG::PowerControl::PowerControlStrategy power_strategy(1288.5f);    // 超电管理

// 电机模型参数
float coefficients3508[6] = {2.144951f, -0.002828f, 0.000025f, 0.016525f, 0.115369f, 0.000015f};


/************************************* 底盘控制逻辑 *************************************/
/*
 * 停止底盘并清空轮速环输出。
 */
void StopChassis()
{
    for (int i = 0; i < 4; i++)
    {
        wheel_pid[i].reset();   
        chassis_output.out_wheel[i] = 0.0f;
    }
}

/*
 * 执行底盘运动学解算和四个轮子的速度闭环。
 */
void RunChassisController(const ChassisTarget& target)
{
    // 前向运动学
    omni_fk.OmniForKinematics(Motor3508.getVelocityRads(1),
                              Motor3508.getVelocityRads(2),
                              Motor3508.getVelocityRads(3),
                              Motor3508.getVelocityRads(4));

    // 逆运动学
    omni_ik.OmniInvKinematics(target.target_translation_x,
                              target.target_translation_y,
                              target.target_rotation,
                              0.0f,
                              1.0f,
                              1.0f);

    // 输出
    for (int i = 0; i < 4; i++)
    {
        wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRads(i + 1));
        chassis_output.out_wheel[i] = wheel_pid[i].getOutput();
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
}

/*
 * 执行超电和裁判系统功率限制。
 */
void ApplyChassisPowerLimit(const OperatorInput& input)
{
    const bool is_supercap_online = supercap.isConnected(); // 超电是否断联
    const bool is_referee_online = !RM_RefereeSystem::RM_RefereeSystemDir(); // 裁判系统是否断联

    // 更新超电和裁判系统在线状态
    supercap.setSupercapOnline(is_supercap_online);
    supercap.setRefereeOnline(is_referee_online);

    // 更新超电策略
    power_strategy.Update(is_supercap_online,
                          is_referee_online,
                          static_cast<float>(ext_power_heat_data_0x0201.chassis_power_limit),
                          ext_power_heat_data_0x0202.chassis_power_buffer,
                          supercap.GetCurrentEnergy(),
                          input.force_referee_untrusted);

    // 获取当前策略的功率上限和剩余能量
    const float input_limit = power_strategy.GetInputLimit();
    const float input_energy = power_strategy.GetInputEnergy();

    // 计算能量环pid
    energy_pid[0].UpDate(sqrtf(energy_ring.GetAbundanceLine()), sqrtf(input_energy));
    energy_pid[1].UpDate(sqrtf(energy_ring.GetPovertyLine()), sqrtf(input_energy));
    // 更新能量环
    energy_ring.energyring(energy_pid[0].getOutput(),
                           energy_pid[1].getOutput(),
                           input_limit,
                           input_energy,
                           input.boost,
                           input.charge);
    
    // 获取当前能量环计算出的功率上限
    const float power_max = energy_ring.GetPowerMax();

    // 更新超电模块信息：开/关，等级功率，缓冲能量
    supercap.setInstruction(0);
    supercap.setRatedPower(input_limit);
    supercap.setBufferEnergy(ext_power_heat_data_0x0202.chassis_power_buffer);

    // 必要数据：电流， 速度
    float i3508[4], i_other[4], v3508[4];

    // 更新必要数据
    for (int i = 0; i < 4; i++)
    {
        i3508[i] = chassis_output.out_wheel[i] * 20.0f / 16384.0f;
        i_other[i] = 0.0f;
        v3508[i] = Motor3508.getVelocityRads(i + 1) * (268.0f / 17.0f);
    }

    // 功率重分配：电流衰减法
    power3508.DecayingCurrent(i3508, v3508, coefficients3508, i_other, 0.0f, power_max);

    // 更新底盘输出电流
    for (int i = 0; i < 4; i++)
    {
        chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f / 20.0f;
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
}
} /* namespace */


/*
 * 初始化底盘控制逻辑。
 */
void InitChassisControl()
{
    chassis_fsm.Init();
    InitChassisPlanner();
}

/*
 * 底盘控制主逻辑。
 */
void ChassisControlLoop(bool is_online, const OperatorInput& input)
{
    // 检测事件
    const ChassisEvent event = DetectChassisEvent(input, is_online);
    // 处理事件
    chassis_fsm.Dispatch(event);

    // 若底盘不在线，强制进入 STOP 状态
    if (!is_online)
    {
        chassis_fsm.ForceStop();
    }

    // 设置底盘目标
    chassis_target = PlanChassisTarget(chassis_fsm.Get_Now_State(), input);

    // 运行底盘控制逻辑
    switch (chassis_fsm.Get_Now_State())
    {
        case STOP:
            StopChassis();
            break;

        case FOLLOW:
        case NOTFOLLOW:
            RunChassisController(chassis_target);
            ApplyChassisPowerLimit(input);
            break;

        default:
            StopChassis();
            break;
    }
}
