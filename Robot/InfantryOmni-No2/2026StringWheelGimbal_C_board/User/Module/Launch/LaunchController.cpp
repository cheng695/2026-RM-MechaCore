#include "LaunchController.hpp"

#include "Module/Launch/LaunchPlanner.hpp"
#include "Module/RobotContext.hpp"
#include "Task/ControlTask.hpp"

#include <cmath>

/*
 * 发射机构控制器 PID。
 *
 * dial_pid：拨盘速度模式使用。
 * dial_angle_pid + dial_velocity_pid：拨盘角度模式使用。
 * surgewheel_pid：两个摩擦轮速度闭环使用。
 */
ALG::PID::PID dial_pid(10.0f, 0.0f, 0.0f, 16384.0f, 0.0f, 0.0f);
ALG::PID::PID dial_angle_pid(10.0f, 0.0f, 0.0f, 12000.0f, 0.0f, 0.0f);
ALG::PID::PID dial_velocity_pid(10.0f, 0.0f, 0.0f, 16384.0f, 0.0f, 0.0f);
ALG::PID::PID surgewheel_pid[2] = {
    ALG::PID::PID(10.0f, 0.0f, 0.0f, 16384.0f, 0.0f, 0.0f),
    ALG::PID::PID(10.0f, 0.0f, 0.0f, 16384.0f, 0.0f, 0.0f)};

namespace
{
constexpr float JAM_POSITION_ERROR_THRESHOLD = 120.0f * 36.0f;
constexpr float JAM_MOVEMENT_THRESHOLD = 120.0f * 36.0f;
constexpr uint32_t JAM_CHECK_INTERVAL_MS = 300U;

/**
 * @brief 清空发射机构电机输出。
 */
void ApplyLaunchStopOutput()
{
    launch_output.out_surgewheel[0] = 0.0f;
    launch_output.out_surgewheel[1] = 0.0f;
    launch_output.out_dial = 0.0f;
}

/**
 * @brief 根据目标转速控制摩擦轮。
 * @param target 发射机构规划目标。
 */
void ApplySurgewheelVelocity(const LaunchTarget& target)
{
    for (int i = 0; i < 2; ++i)
    {
        surgewheel_pid[i].UpDate(target.target_surgewheel[i], Motor3508.getVelocityRpm(i + 2));
        launch_output.out_surgewheel[i] = surgewheel_pid[i].getOutput();
    }
}

/**
 * @brief 拨盘速度控制。
 * @param target 发射机构规划目标。
 *
 * STOP、CEASEFIRE、AUTO 都可以走速度控制：目标为 0 时拨盘停止，
 * AUTO 下 Planner 会持续推进 target_dial，也可以改为角度控制。
 */
void ApplyDialVelocity(const LaunchTarget& target)
{
    dial_pid.UpDate(target.target_dial, Motor3508.getVelocityRpm(1));
    launch_output.out_dial = dial_pid.getOutput();
}

/**
 * @brief 拨盘角度控制。
 * @param target 发射机构规划目标。
 *
 * ONLY 单发模式需要拨盘前进一个固定槽位，因此使用角度外环 + 速度内环。
 */
void ApplyDialAngle(const LaunchTarget& target)
{
    dial_angle_pid.UpDate(target.target_dial, Motor3508.getAddAngleDeg(1));
    dial_velocity_pid.UpDate(dial_angle_pid.getOutput(), Motor3508.getVelocityRpm(1));
    launch_output.out_dial = dial_velocity_pid.getOutput();
}

/**
 * @brief 发射机构速度模式输出。
 * @param target 发射机构规划目标。
 */
void ApplyLaunchVelocityTarget(const LaunchTarget& target)
{
    ApplySurgewheelVelocity(target);
    ApplyDialVelocity(target);
}

/**
 * @brief 发射机构单发角度模式输出。
 * @param target 发射机构规划目标。
 */
void ApplyLaunchAngleTarget(const LaunchTarget& target)
{
    ApplySurgewheelVelocity(target);
    ApplyDialAngle(target);
}

/**
 * @brief 卡弹退弹输出。
 * @param target 发射机构规划目标。
 *
 * JAM 状态下摩擦轮保持夹弹目标，拨盘直接输出退弹目标值。
 */
void ApplyLaunchJamTarget(const LaunchTarget& target)
{
    ApplySurgewheelVelocity(target);
    launch_output.out_dial = target.target_dial;
}
}

/**
 * @brief 卡弹检测。
 * @return true 表示检测到拨盘可能卡弹。
 *
 * 判断逻辑：如果拨盘目标和反馈误差长期很大，并且一段时间内反馈角度几乎不动，
 * 就认为拨盘被弹丸卡住，需要进入 JAM 退弹状态。
 */
bool is_jamming()
{
    static uint32_t last_check_time = 0;
    static float last_angle = 0.0f;

    const float current_angle = Motor3508.getAddAngleDeg(1);
    const float err = std::fabs(launch_planner_target.target_dial - current_angle);

    if (err > JAM_POSITION_ERROR_THRESHOLD)
    {
        const uint32_t now = HAL_GetTick();

        if (last_check_time == 0)
        {
            last_check_time = now;
            last_angle = current_angle;
            return false;
        }

        if (now - last_check_time > JAM_CHECK_INTERVAL_MS)
        {
            if (std::fabs(current_angle - last_angle) < JAM_MOVEMENT_THRESHOLD)
            {
                last_check_time = 0;
                return true;
            }

            last_check_time = now;
            last_angle = current_angle;
        }
    }
    else
    {
        last_check_time = 0;
    }

    return false;
}

/**
 * @brief 执行一次发射机构控制。
 * @param input 本周期发射机构输入。
 *
 * Controller 先调用 Planner 更新目标，再根据发射状态机当前状态选择控制方式。
 */
void RunLaunchController(const LaunchInput& input)
{
    launch_planner_target = PlanLaunchTarget(launch_fsm.Get_Now_State(),
                                             input,
                                             launch_planner_target);

    switch (launch_fsm.Get_Now_State())
    {
    case LAUNCH_STOP:
        ApplyLaunchStopOutput();
        break;

    case LAUNCH_CEASEFIRE:
    case LAUNCH_AUTO:
        ApplyLaunchVelocityTarget(launch_planner_target);
        break;

    case LAUNCH_ONLY:
        ApplyLaunchAngleTarget(launch_planner_target);
        break;

    case LAUNCH_JAM:
        ApplyLaunchJamTarget(launch_planner_target);
        break;

    default:
        ApplyLaunchStopOutput();
        break;
    }
}
