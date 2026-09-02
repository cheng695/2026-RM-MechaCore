#include "GimbalController.hpp"

#include "Module/Gimbal/GimbalEventDetector.hpp"
#include "Module/Gimbal/GimbalInputBuilder.hpp"
#include "Module/Gimbal/GimbalPlanner.hpp"
#include "Module/RobotContext.hpp"
#include "Task/ControlTask.hpp"
#include "core/Alg/Feedforward/Feedforward.hpp"
#include "core/Alg/Filter/Filter.hpp"
#include "core/Alg/PID/pid.hpp"
#include "core/Alg/VMC/VMC.hpp"

#include <algorithm>
#include <cmath>

/* UDE 前后级 TD 滤波器：input 滤 UDE 输入，out 滤 UDE 输出。 */
TDFilter ude_filter_input(40.0f, 0.001f);
TDFilter ude_filter_out(20.0f, 0.001f);

/* 云台前馈对象。J2 当前只使用重力前馈；yaw 使用惯量/摩擦前馈和 UDE 扰动补偿。 */
Alg::Feedforward::Gravity gravity_forward_J2(0.25f, -90.0f);
Alg::Feedforward::Gravity gravity_forward_J1(0.0f, 0.0f);
Alg::Feedforward::GimbalFullCompensation gimbal_yaw(0.00005f, 0.001f, 0.0001f, 0.002f);
Alg::Feedforward::UDE ude_yaw(0.1f, 1.0f);

/* J1 使用 VMC 输出力矩，MIT 的 J1 位置/速度项保持为 0。 */
ALG::VMC::VMC pitch_J1_vmc(0.12f, 0.12f, 100.0f, 100.0f, 2.0f, 2.0f);

/* 手动模式 yaw 双环：外环自己算速度期望，内环由达妙 MIT 的 Kp/Kd 完成。 */
ALG::PID::PID yaw_angle_pid(20.0f, 0.0f, 0.0f, 12.5664f, 0.0f, 0.0f);
ALG::PID::PID yaw_velocity_pid(0.0f, 0.0f, 1.0f, 16384.0f, 0.0f, 0.0f);

/* 视觉模式 yaw 双环：使用普通 yaw 角，不使用叠加 yaw 角。 */
ALG::PID::PID yaw_angle_vision_pid(20.0f, 0.0f, 0.0f, 12.5664f, 0.0f, 0.0f);
ALG::PID::PID yaw_velocity_vision_pid(0.0f, 0.0f, 1.0f, 16384.0f, 0.0f, 0.0f);

/* 手动模式 J2 双环：外环使用 IMU pitch，内环由达妙 MIT 的 Kp/Kd 完成。 */
ALG::PID::PID pitch_angle_pid(20.0f, 0.0f, 0.0f, 12.5664f, 0.0f, 0.0f);
ALG::PID::PID pitch_velocity_pid(0.0f, 0.0f, 1.0f, 16384.0f, 0.0f, 0.0f);

/* 视觉模式 J2 双环：和手动模式分开调参。 */
ALG::PID::PID pitch_angle_vision_pid(20.0f, 0.0f, 0.0f, 12.5664f, 0.0f, 0.0f);
ALG::PID::PID pitch_velocity_vision_pid(0.0f, 0.0f, 1.0f, 16384.0f, 0.0f, 0.0f);

/* 折叠/起立过程专用 PID。 */
ALG::PID::PID yaw_transform_pid(0.0f, 0.0f, 1.0f, 16384.0f, 0.0f, 0.0f);
ALG::PID::PID pitch_transform_angle_pid(20.0f, 0.0f, 0.0f, 12.5664f, 0.0f, 0.0f);
ALG::PID::PID pitch_transform_velocity_pid(0.0f, 0.0f, 1.0f, 16384.0f, 0.0f, 0.0f);

namespace
{
constexpr float PI_VALUE = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI_VALUE / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI_VALUE;
constexpr float RADS_TO_RPM = 60.0f / (2.0f * PI_VALUE);
constexpr float YAW_TRANSITION_TOLERANCE = 3.0f * DEG_TO_RAD;
constexpr float PITCH_TRANSITION_TOLERANCE = 3.0f * DEG_TO_RAD;

/**
 * @brief 将角度误差归一化到 [-pi, pi]。
 * @param angle 原始角度误差，单位 rad。
 * @return 归一化后的最短角度误差，单位 rad。
 */
float NormalizeAngle(float angle)
{
    while (angle > PI_VALUE)
    {
        angle -= 2.0f * PI_VALUE;
    }

    while (angle < -PI_VALUE)
    {
        angle += 2.0f * PI_VALUE;
    }

    return angle;
}

/**
 * @brief 把普通绝对角目标映射到当前反馈附近。
 * @param target 普通角度目标，单位 rad。
 * @param feedback 普通角度反馈，单位 rad。
 * @return 与反馈最接近的一圈内等效目标，单位 rad。
 *
 * 视觉和折叠 yaw 使用普通角度，普通角度会在 pi/-pi 附近跳变。
 * 该函数可以让控制器走最短角度误差，避免边界处反向绕圈。
 */
float BuildShortestAngleTarget(float target, float feedback)
{
    return feedback + NormalizeAngle(target - feedback);
}

/**
 * @brief 判断角度目标是否到位。
 * @param target 目标角度，单位 rad。
 * @param feedback 反馈角度，单位 rad。
 * @param tolerance 允许误差，单位 rad。
 * @return true 表示目标已到位。
 */
bool IsNearAngle(float target, float feedback, float tolerance)
{
    return std::fabs(NormalizeAngle(target - feedback)) <= tolerance;
}

/**
 * @brief 判断当前是否应该使用折叠/起立专用控制。
 * @param state 当前稳定状态。
 * @param transition 当前半自动过渡阶段。
 * @return true 表示处于折叠、起立或折叠保持状态。
 */
bool IsTransformControl(Enum_Gimbal_States state, GimbalTransition transition)
{
    return transition != GimbalTransition::None || state == DOWN;
}

/**
 * @brief 清空云台所有 PID 的历史状态。
 *
 * 状态切换、半自动动作完成或初始化时调用，避免旧模式的积分项和微分项
 * 影响新模式，尤其是 VISION 切回 MANUAL 时的 yaw 目标跳变问题。
 */
void ResetGimbalPid()
{
    yaw_angle_pid.reset();
    yaw_velocity_pid.reset();
    yaw_angle_vision_pid.reset();
    yaw_velocity_vision_pid.reset();
    pitch_angle_pid.reset();
    pitch_velocity_pid.reset();
    pitch_angle_vision_pid.reset();
    pitch_velocity_vision_pid.reset();
    yaw_transform_pid.reset();
    pitch_transform_angle_pid.reset();
    pitch_transform_velocity_pid.reset();
}

/**
 * @brief 计算 J2 重力前馈。
 * @param input 本周期云台输入，使用 IMU pitch 作为末端姿态反馈。
 * @return 限幅后的 J2 前馈力矩。
 */
float CalcJ2GravityFeedforward(const GimbalInput& input)
{
    gravity_forward_J2.GravityFeedforward(input.current_pitch * RAD_TO_DEG);
    return std::clamp(gravity_forward_J2.getFeedforward(), -3.0f, 3.0f);
}

/**
 * @brief 计算 yaw 力矩前馈。
 * @param target_velocity yaw 外环输出的速度期望，单位 rad/s。
 * @param input 本周期云台输入，使用 IMU yaw 角速度作为速度反馈。
 * @return yaw 前馈力矩，写入达妙 MIT 的 torque 项。
 *
 * gimbal_yaw 是常规 yaw 前馈，手动和视觉模式都使用。
 * ude_yaw 只在底盘小陀螺时叠加，用来补偿底盘旋转给云台 yaw 带来的扰动。
 * UDE 的输入和输出分别经过 TD 滤波，避免扰动估计直接把尖峰带进力矩项。
 */
float CalcYawFeedforward(float target_velocity, const GimbalInput& input)
{
    const float feedback_velocity_rpm = input.current_yaw_velocity * RADS_TO_RPM;
    const float target_velocity_rpm = target_velocity * RADS_TO_RPM;

    gimbal_yaw.MomentOfInertiaTuning(feedback_velocity_rpm,
                                     target_velocity_rpm);
    float feedforward = gimbal_yaw.getTorque();

    if (input.chassis_spin)
    {
        /*
         * UDE 只在底盘小陀螺时启用。
         * 这里用常规 yaw 前馈作为 UDE 输入，用 IMU yaw 角速度作为状态导数。
         * 后续如果可以拿到底盘实际自旋角速度，可以把这里改成相对 yaw 速度，
         * 扰动估计会更接近真实底盘扰动。
         */
        const float ude_input = ude_filter_input.filter(feedforward);

        ude_yaw.UDE_Update(ude_input, input.current_yaw_velocity);

        const float ude_output = ude_filter_out.filter(ude_yaw.getOutput());
        feedforward += ude_output;
    }
    else
    {
        ude_yaw.ResetState();
    }

    return feedforward;
}

/**
 * @brief STOP 状态输出。
 * @param input 本周期云台输入。
 *
 * STOP 下不输出主动控制力矩和 MIT 刚度，只把当前位置写入目标字段。
 * 这样重新使能或切回控制状态时，调试变量不会停留在旧目标上。
 */
void ApplyStopOutput(const GimbalInput& input)
{
    gimbal_output.out_yaw_angle = input.current_yaw;
    gimbal_output.out_yaw_velocity = 0.0f;
    gimbal_output.out_yaw_torque = 0.0f;
    gimbal_k.yaw_Kp = 0.0f;
    gimbal_k.yaw_Kd = 0.0f;

    gimbal_output.out_J1_angle = input.current_J1;
    gimbal_output.out_J1_velocity = 0.0f;
    gimbal_output.out_J1_torque = 0.0f;
    gimbal_k.J1_Kp = 0.0f;
    gimbal_k.J1_Kd = 0.0f;

    gimbal_output.out_J2_angle = input.current_J2;
    gimbal_output.out_J2_velocity = 0.0f;
    gimbal_output.out_J2_torque = 0.0f;
    gimbal_k.J2_Kp = 0.0f;
    gimbal_k.J2_Kd = 0.0f;
}

/**
 * @brief 使用 VMC 生成 J1 控制输出。
 * @param target Planner 生成的 J1/J2 目标。
 * @param input J1/J2 当前角度和角速度反馈。
 *
 * J1 全程由 VMC 管理，因此 MIT 的 J1 位置环 Kp/Kd 置 0。
 * MotorTask 会把 VMC 计算出的力矩写入达妙 MIT 控制帧的 torque 项。
 */
void ApplyJ1Vmc(const GimbalTarget& target, const GimbalInput& input)
{
    pitch_J1_vmc.Settheta(input.current_J1 * RAD_TO_DEG,
                          input.current_J2 * RAD_TO_DEG);
    pitch_J1_vmc.Settheta_dot(input.current_J1_velocity,
                              input.current_J2_velocity);
    pitch_J1_vmc.VMC_Update(target.target_J2 * RAD_TO_DEG,
                            target.target_J1 * RAD_TO_DEG);

    gimbal_output.out_J1_angle = target.target_J1;  // 调试目标值，J1 实际由 VMC 力矩控制。
    gimbal_output.out_J1_velocity = 0.0f;
    gimbal_output.out_J1_torque = std::clamp(pitch_J1_vmc.GetT1(), -9.0f, 9.0f);
    gimbal_k.J1_Kp = 0.0f;
    gimbal_k.J1_Kd = 0.0f;
}

/**
 * @brief 手动模式 yaw 闭环。
 * @param target Planner 生成的叠加 yaw 目标。
 * @param input 当前叠加 yaw 反馈和 yaw 角速度反馈。
 *
 * 手动模式使用叠加角，目标和反馈都可以超过一圈。
 * 外环 yaw_angle_pid 输出速度期望；MIT 内环 Kp/Kd 从 yaw_velocity_pid 读取；
 * torque 项叠加 yaw 前馈。
 */
void ApplyManualYaw(const GimbalTarget& target, const GimbalInput& input)
{
    yaw_angle_pid.UpDate(target.target_yaw, input.current_yaw);
    const float target_velocity = yaw_angle_pid.getOutput();

    gimbal_output.out_yaw_angle = 0.0f;
    gimbal_output.out_yaw_velocity = target_velocity;
    gimbal_output.out_yaw_torque = CalcYawFeedforward(target_velocity, input);
    gimbal_k.yaw_Kp = yaw_velocity_pid.getK(0);
    gimbal_k.yaw_Kd = yaw_velocity_pid.getK(2);

    /*
     * 备用实验方案：手动 yaw 纯自写双环，只通过 MIT torque 项输出。
     * 外环仍然使用叠加 yaw 角；内环使用 IMU yaw 角速度反馈。
     * 前馈不变，仍然叠加 CalcYawFeedforward()。
     *
     * yaw_angle_pid.UpDate(target.target_yaw, input.current_yaw);
     * const float target_velocity = yaw_angle_pid.getOutput();
     * yaw_velocity_pid.UpDate(target_velocity, input.current_yaw_velocity);
     *
     * gimbal_output.out_yaw_angle = 0.0f;
     * gimbal_output.out_yaw_velocity = 0.0f;
     * gimbal_output.out_yaw_torque =
     *     yaw_velocity_pid.getOutput() + CalcYawFeedforward(target_velocity, input);
     * gimbal_k.yaw_Kp = 0.0f;
     * gimbal_k.yaw_Kd = 0.0f;
     */
}

/**
 * @brief 视觉模式 yaw 闭环。
 * @param target Planner 生成的视觉 yaw 目标。
 * @param input 当前普通 yaw 角反馈和 yaw 角速度反馈。
 *
 * 视觉模式只使用普通 yaw 角，不使用叠加 yaw 角。
 * 外环 yaw_angle_vision_pid 输出速度期望；MIT 内环 Kp/Kd 从 yaw_velocity_vision_pid 读取；
 * torque 项叠加 yaw 前馈。
 */
void ApplyVisionYaw(const GimbalTarget& target, const GimbalInput& input)
{
    const float yaw_target = BuildShortestAngleTarget(target.target_yaw,
                                                     input.current_yaw_angle);
    yaw_angle_vision_pid.UpDate(yaw_target, input.current_yaw_angle);
    const float target_velocity = yaw_angle_vision_pid.getOutput();

    gimbal_output.out_yaw_angle = 0.0f;
    gimbal_output.out_yaw_velocity = target_velocity;
    gimbal_output.out_yaw_torque = CalcYawFeedforward(target_velocity, input);
    gimbal_k.yaw_Kp = yaw_velocity_vision_pid.getK(0);
    gimbal_k.yaw_Kd = yaw_velocity_vision_pid.getK(2);

    /*
     * 备用实验方案：视觉 yaw 纯自写双环，只通过 MIT torque 项输出。
     * 外环使用普通 yaw 角，目标由 BuildShortestAngleTarget() 处理到当前反馈附近。
     * 前馈不变，仍然叠加 CalcYawFeedforward()。
     *
     * yaw_angle_vision_pid.UpDate(yaw_target, input.current_yaw_angle);
     * const float target_velocity = yaw_angle_vision_pid.getOutput();
     * yaw_velocity_vision_pid.UpDate(target_velocity, input.current_yaw_velocity);
     *
     * gimbal_output.out_yaw_angle = 0.0f;
     * gimbal_output.out_yaw_velocity = 0.0f;
     * gimbal_output.out_yaw_torque =
     *     yaw_velocity_vision_pid.getOutput() + CalcYawFeedforward(target_velocity, input);
     * gimbal_k.yaw_Kp = 0.0f;
     * gimbal_k.yaw_Kd = 0.0f;
     */
}

/**
 * @brief 折叠/起立过程 yaw 闭环。
 * @param target Planner 生成的变形 yaw 目标。
 * @param input 当前普通 yaw 角反馈。
 *
 * 当前启用方案：直接使用达妙 MIT 的位置/速度控制。
 * Planner 给出固定折叠角或当前保持角，本函数把普通角目标映射到当前反馈附近，
 * 降低 pi/-pi 边界附近的跳变风险。
 */
void ApplyTransformYaw(const GimbalTarget& target, const GimbalInput& input)
{
    const float yaw_target = BuildShortestAngleTarget(target.target_yaw,
                                                     input.current_yaw_angle);

    gimbal_output.out_yaw_angle = yaw_target;
    gimbal_output.out_yaw_velocity = 0.0f;
    gimbal_output.out_yaw_torque = 0.0f;
    gimbal_k.yaw_Kp = yaw_transform_pid.getK(0);
    gimbal_k.yaw_Kd = yaw_transform_pid.getK(2);

    /*
     * 备用实验方案：纯自写双环，只把最终力矩交给 MIT，MIT 的位置/速度项全部置 0。
     * 如果实测发现达妙内置位置环在折叠 yaw 时超调明显，可以改成下面这种方式。
     *
     * yaw_angle_pid.UpDate(yaw_target, input.current_yaw_angle);
     * yaw_velocity_pid.UpDate(yaw_angle_pid.getOutput(), 0.0f);
     * gimbal_output.out_yaw_angle = 0.0f;
     * gimbal_output.out_yaw_velocity = 0.0f;
     * gimbal_output.out_yaw_torque = yaw_velocity_pid.getOutput();
     * gimbal_k.yaw_Kp = 0.0f;
     * gimbal_k.yaw_Kd = 0.0f;
     */
}

/**
 * @brief 手动模式 J2 闭环。
 * @param target Planner 生成的 J2 目标。
 * @param input 当前 IMU pitch 反馈。
 *
 * 手动模式下 J2 外环使用 pitch_angle_pid，反馈使用 IMU pitch。
 * MIT 内环 Kp/Kd 从 pitch_velocity_pid 读取，torque 项叠加 J2 重力前馈。
 */
void ApplyManualJ2(const GimbalTarget& target, const GimbalInput& input)
{
    pitch_angle_pid.UpDate(target.target_J2, input.current_pitch);

    gimbal_output.out_J2_angle = 0.0f;
    gimbal_output.out_J2_velocity = pitch_angle_pid.getOutput();
    gimbal_output.out_J2_torque = CalcJ2GravityFeedforward(input);
    gimbal_k.J2_Kp = pitch_velocity_pid.getK(0);
    gimbal_k.J2_Kd = pitch_velocity_pid.getK(2);

    /*
     * 备用实验方案：手动 J2 纯自写双环，只通过 MIT torque 项输出。
     * 外环使用 IMU pitch；内环使用 J2 电机角速度反馈。
     * 重力前馈不变，仍然叠加 CalcJ2GravityFeedforward()。
     *
     * pitch_angle_pid.UpDate(target.target_J2, input.current_pitch);
     * pitch_velocity_pid.UpDate(pitch_angle_pid.getOutput(), input.current_J2_velocity);
     *
     * gimbal_output.out_J2_angle = 0.0f;
     * gimbal_output.out_J2_velocity = 0.0f;
     * gimbal_output.out_J2_torque =
     *     pitch_velocity_pid.getOutput() + CalcJ2GravityFeedforward(input);
     * gimbal_k.J2_Kp = 0.0f;
     * gimbal_k.J2_Kd = 0.0f;
     */
}

/**
 * @brief 视觉模式 J2 闭环。
 * @param target Planner 生成的视觉 pitch/J2 目标。
 * @param input 当前 IMU pitch 反馈。
 *
 * 视觉模式下 J2 外环使用 pitch_angle_vision_pid。
 * MIT 内环 Kp/Kd 从 pitch_velocity_vision_pid 读取，torque 项叠加 J2 重力前馈。
 */
void ApplyVisionJ2(const GimbalTarget& target, const GimbalInput& input)
{
    pitch_angle_vision_pid.UpDate(target.target_J2, input.current_pitch);

    gimbal_output.out_J2_angle = 0.0f;
    gimbal_output.out_J2_velocity = pitch_angle_vision_pid.getOutput();
    gimbal_output.out_J2_torque = CalcJ2GravityFeedforward(input);
    gimbal_k.J2_Kp = pitch_velocity_vision_pid.getK(0);
    gimbal_k.J2_Kd = pitch_velocity_vision_pid.getK(2);

    /*
     * 备用实验方案：视觉 J2 纯自写双环，只通过 MIT torque 项输出。
     * 外环使用视觉 pitch 目标和 IMU pitch 反馈；内环使用 J2 电机角速度反馈。
     * 重力前馈不变，仍然叠加 CalcJ2GravityFeedforward()。
     *
     * pitch_angle_vision_pid.UpDate(target.target_J2, input.current_pitch);
     * pitch_velocity_vision_pid.UpDate(pitch_angle_vision_pid.getOutput(),
     *                                  input.current_J2_velocity);
     *
     * gimbal_output.out_J2_angle = 0.0f;
     * gimbal_output.out_J2_velocity = 0.0f;
     * gimbal_output.out_J2_torque =
     *     pitch_velocity_vision_pid.getOutput() + CalcJ2GravityFeedforward(input);
     * gimbal_k.J2_Kp = 0.0f;
     * gimbal_k.J2_Kd = 0.0f;
     */
}

/**
 * @brief 折叠/起立过程 J2 闭环。
 * @param target Planner 生成的 J2 目标，当前约定始终为 0 rad。
 * @param input 当前 IMU pitch 与 J2 电机反馈。
 *
 * 在折叠全过程、起立全过程以及 DOWN 稳定状态中，J2 目标都是 0 rad，
 * 目的是让末端保持世界坐标系水平。外环使用 IMU pitch，内环交给 MIT。
 */
void ApplyTransformJ2(const GimbalTarget& target, const GimbalInput& input)
{
    pitch_transform_angle_pid.UpDate(target.target_J2, input.current_pitch);

    gimbal_output.out_J2_angle = 0.0f;
    gimbal_output.out_J2_velocity = pitch_transform_angle_pid.getOutput();
    gimbal_output.out_J2_torque = CalcJ2GravityFeedforward(input);
    gimbal_k.J2_Kp = pitch_transform_velocity_pid.getK(0);
    gimbal_k.J2_Kd = pitch_transform_velocity_pid.getK(2);

    /*
     * 备用实验方案：J2 纯自写双环，只输出力矩。
     * 这里速度反馈建议优先使用 J2 电机角速度；如果后续需要世界坐标系 pitch 速度，
     * 可以再从 IMU pitch 角速度补充到 GimbalInput。
     *
     * pitch_transform_velocity_pid.UpDate(pitch_transform_angle_pid.getOutput(),
     *                                     input.current_J2_velocity);
     * gimbal_output.out_J2_angle = 0.0f;
     * gimbal_output.out_J2_velocity = 0.0f;
     * gimbal_output.out_J2_torque =
     *     pitch_transform_velocity_pid.getOutput() + CalcJ2GravityFeedforward(input);
     * gimbal_k.J2_Kp = 0.0f;
     * gimbal_k.J2_Kd = 0.0f;
     */
}

/**
 * @brief 把 Planner 目标转换成 MotorTask 可直接发送的 MIT 输出。
 * @param target Planner 输出的 yaw/J1/J2 目标。
 * @param input 本周期云台输入和反馈。
 *
 * 控制结构：
 * 1. J1 始终由 VMC 输出力矩；
 * 2. MANUAL 使用手动 yaw/J2 外环 PID；
 * 3. VISION 使用视觉 yaw/J2 外环 PID；
 * 4. 折叠、起立和 DOWN 使用变形 yaw/J2 控制参数。
 */
void ApplyGimbalTarget(const GimbalTarget& target, const GimbalInput& input)
{
    const Enum_Gimbal_States state = gimbal_fsm.Get_Now_State();
    const GimbalTransition transition = gimbal_fsm.Get_Transition();

    if (state == STOP && transition == GimbalTransition::None)
    {
        ApplyStopOutput(input);
        return;
    }

    ApplyJ1Vmc(target, input);

    if (IsTransformControl(state, transition))
    {
        ApplyTransformYaw(target, input);
        ApplyTransformJ2(target, input);
        return;
    }

    if (state == VISION)
    {
        ApplyVisionYaw(target, input);
        ApplyVisionJ2(target, input);
        return;
    }

    ApplyManualYaw(target, input);
    ApplyManualJ2(target, input);
}

/**
 * @brief 判断当前半自动过渡目标是否到位。
 * @param target Planner 当前输出目标。
 * @param input 当前反馈。
 * @return true 表示可以通知 FSM 提交到最终稳定状态。
 */
bool IsTransitionTargetReached(const GimbalTarget& target, const GimbalInput& input)
{
    const GimbalTransition transition = gimbal_fsm.Get_Transition();

    if (transition == GimbalTransition::None)
    {
        return false;
    }

    const bool j1_reached = std::fabs(target.target_J1 - input.current_J1) <=
                            PITCH_TRANSITION_TOLERANCE;
    const bool j2_reached = std::fabs(target.target_J2 - input.current_pitch) <=
                            PITCH_TRANSITION_TOLERANCE;

    if (transition == GimbalTransition::StopToDown ||
        transition == GimbalTransition::ManualToDown ||
        transition == GimbalTransition::VisionToDown)
    {
        const bool yaw_reached = IsNearAngle(target.target_yaw,
                                            input.current_yaw_angle,
                                            YAW_TRANSITION_TOLERANCE);
        return yaw_reached && j1_reached && j2_reached;
    }

    return j1_reached && j2_reached;
}

/**
 * @brief 根据在线状态和操作输入更新云台状态机。
 * @param is_online 云台控制所需设备是否在线。
 * @param input 本周期云台输入。
 */
void UpdateGimbalState(bool is_online, const GimbalInput& input)
{
    if (!is_online)
    {
        gimbal_fsm.ForceStop();
        return;
    }

    const GimbalEvent event = DetectGimbalEvent(input.left_switch,
                                                input.right_switch,
                                                is_online,
                                                input.vision_enabled,
                                                input.transform_requested);
    gimbal_fsm.Dispatch(event);
}

/**
 * @brief 半自动动作到位后提交 FSM 最终状态。
 * @param target Planner 当前输出目标。
 * @param input 当前反馈。
 */
void CompleteTransitionIfReached(const GimbalTarget& target, const GimbalInput& input)
{
    if (IsTransitionTargetReached(target, input))
    {
        gimbal_fsm.CompleteTransition();
        ResetGimbalPid();
    }
}
} /* namespace */

/**
 * @brief 初始化云台控制模块。
 *
 * 初始化内容包括云台 FSM、Planner 和 PID 历史项。
 */
void InitGimbalControl()
{
    gimbal_fsm.Init();
    InitGimbalPlanner();
    ResetGimbalPid();
}

/**
 * @brief 云台控制主循环。
 * @param is_online 设备在线状态，由 ControlTask::check_online() 提供。
 * @param input 本周期云台输入，由 GimbalInputBuilder 构造。
 */
void GimbalControlLoop(bool is_online, const GimbalInput& input)
{
    UpdateGimbalState(is_online, input);
    gimbal_fsm.TIM_Update();

    gimbal_planner_target = PlanGimbalTarget(gimbal_fsm.Get_Now_State(),
                                             gimbal_fsm.Get_Transition(),
                                             input,
                                             gimbal_planner_target);
    ApplyGimbalTarget(gimbal_planner_target, input);
    CompleteTransitionIfReached(gimbal_planner_target, input);
}

/**
 * @brief 构造运行时输入，并执行一次云台控制循环。
 */
void RunGimbalController()
{
    const GimbalInput input = BuildRuntimeGimbalInput();
    GimbalControlLoop(check_online(), input);
}
