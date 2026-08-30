#include "RobotContext.hpp"

namespace
{
constexpr float PI_VALUE = 3.1415926535897932384626433832795f;
} /* namespace */

// 遥控器
BSP::REMOTE_CONTROL::RemoteController DT7;

// 电机
BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);

// 板间通讯
BoardCommunication Cboard;

// 超电
Supercapacitor supercap(0x666);

// 输入
OperatorInput operator_input;

// 底盘状态机
Chassis_FSM chassis_fsm;

// 底盘目标和输出
ChassisTarget chassis_target;
ChassisOutput chassis_output;

/*
 * 底盘全向轮几何参数。轮子方位角 与 轮子坐标 的关系如下：
 *
 * IK 和 FK 共用这一份参数，避免目标解算和速度反馈使用不同模型。
 */
float chassis_wheel_azimuth[4] = {
    -PI_VALUE / 4.0f,
    -3.0f * PI_VALUE / 4.0f,
    3.0f * PI_VALUE / 4.0f,
    PI_VALUE / 4.0f};

float chassis_wheel_direction[4] = {
    -3.0f * PI_VALUE / 4.0f,
    3.0f * PI_VALUE / 4.0f,
    PI_VALUE / 4.0f,
    -PI_VALUE / 4.0f};

Alg::CalculationBase::Omni_FK omni_fk(0.24f, 0.07f, 4.0f, chassis_wheel_azimuth, chassis_wheel_direction);

ALG::PowerControl::EnergyRing energy_ring(1288.5f, 250.0f, 48.0f);
