#include "ChassisPlanner.hpp"
#include "RobotContext.hpp"
#include "SlopePlanning.hpp"
#include "pid.hpp"

#include <cmath>

namespace
{
constexpr float PI_VALUE = 3.1415926535897932384626433832795f;
constexpr float FOLLOW_STANDARD_YAW = -2.39347f;    // 正方向
constexpr float REMOTE_ROTATE_DEADZONE = 0.05f;     // 遥控器死区
constexpr float CHASSIS_TRANSLATION_GAIN = 4.5f;    // 平移增益
constexpr float CHASSIS_ROTATION_GAIN = 13.0f;      // 旋转增益
constexpr float VEHICLE_TURN_RADIUS = 0.24f;        // 底盘转弯半径 (m)

// 底盘跟随pid
ALG::PID::PID follow_pid(8.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);

// 斜坡规划
Alg::Utility::SlopePlanning omni_target[2] = {
    Alg::Utility::SlopePlanning(0.015f, 0.015f),
    Alg::Utility::SlopePlanning(0.015f, 0.015f)};

/*
 * 将云台坐标系下的平移输入旋转到底盘坐标系。
 * @param theta 云台坐标系下的旋转角度。
 * @param vx 输入的平移X轴。
 * @param vy 输入的平移Y轴。
 * @param phi 正方向偏置角
 * @param out_vx 输出的底盘坐标系下的平移X轴。
 * @param out_vy 输出的底盘坐标系下的平移Y轴
 * @param psi 小陀螺行进补偿相位。
 */
void CalculateTranslationXY(float theta, float vx, float vy, float phi, float* out_vx, float* out_vy, float psi)
{
    theta += phi;
    theta = fmod(theta, 2.0f * PI_VALUE);

    if (theta > PI_VALUE)
    {
        theta -= 2.0f * PI_VALUE;
    }
    else if (theta < -PI_VALUE)
    {
        theta += 2.0f * PI_VALUE;
    }

    const float s = sinf(theta + psi);
    const float c = cosf(theta + psi);
    const float raw_vx = CHASSIS_TRANSLATION_GAIN * vy;
    const float raw_vy = CHASSIS_TRANSLATION_GAIN * vx;

    *out_vx = raw_vx * c + raw_vy * s;
    *out_vy = raw_vx * -s + raw_vy * c;
}

/*
 * 计算跟随模式下的底盘旋转目标。
 */
float CalculateFollowRotation()
{
    // 计算云台与正方向之间的偏差角度
    float follow_error = FOLLOW_STANDARD_YAW - Cboard.GetYawAngle();

    // 将角度误差规划到 [-π, π] 范围内
    while (follow_error > 1.5707963267f)
    {
        follow_error -= 2.0f * 1.5707963267f;
    }

    while (follow_error < -1.5707963267f)
    {
        follow_error += 2.0f * 1.5707963267f;
    }

    if (fabs(follow_error) < 0.1f)
    {
        follow_error = 0.0f;
    }

    // 使用 PID 控制器计算旋转输出
    follow_pid.UpDate(follow_error, 0.0f);
    return follow_pid.getOutput();
}

/*
 * 限制平移和旋转的合成目标，避免运动学解算后轮速超过物理上限。
 */
void LimitChassisVector(float& vx_norm, float& vy_norm, float& vw_norm)
{
    constexpr float max_linear_vel = CHASSIS_TRANSLATION_GAIN;  // 最大线速度 (m/s)
    constexpr float max_rotation_vel = CHASSIS_ROTATION_GAIN;   // 最大角速度 (rad/s)
    constexpr float motor_physical_limit = 5.2f;                // 最大轮速限制 (m/s)，对应电机物理极限

    const float target_vx = vx_norm * max_linear_vel;     // 目标线速度
    const float target_vy = vy_norm * max_linear_vel;     // 目标线速度
    const float target_vw = vw_norm * max_rotation_vel;   // 旋转速度
    const float vw_equivalent_linear = fabs(target_vw) * VEHICLE_TURN_RADIUS; // 旋转速度等效线速度
    const float v_translation_magnitude = sqrtf(target_vx * target_vx + target_vy * target_vy); // 平移线速度
    const float total_required_speed = v_translation_magnitude + vw_equivalent_linear;  // 总需求速度

    // 如果总需求速度超过电机物理极限，则按比例缩放平移和旋转速度
    if (total_required_speed > motor_physical_limit)
    {
        const float scale = motor_physical_limit / total_required_speed;
        vx_norm *= scale;
        vy_norm *= scale;
        vw_norm *= scale;
    }
}

/*
 * 判断当前是否小陀螺
 */
bool IsManualRotateRequested(const OperatorInput& input, float rotate_input)
{
    if (input.keyboard_mode)
    {
        return input.spin;
    }

    return fabs(rotate_input) > REMOTE_ROTATE_DEADZONE;
}
} /* namespace */


/**
 * @brief 初始化底盘规划模块。
 */
void InitChassisPlanner()
{
    follow_pid.reset();
}


/**
 * @brief 计算底盘运动目标。
 * @param state 当前底盘状态。
 * @param input 输入数据。
 * @return 底盘运动目标。
 */
ChassisTarget PlanChassisTarget(Enum_Chassis_States state, const OperatorInput& input)
{
    // 底盘运动目标
    ChassisTarget target;

    // 若底盘不处于跟随或不跟随状态，则直接返回默认目标
    if (state != FOLLOW && state != NOTFOLLOW)
    {
        return target;
    }

    // 解析输入
    float vx_handle = input.move_x;
    float vy_handle = input.move_y;
    float vw_handle = input.rotate;

    // 遥控器模式下，不允许小陀螺 或者 滚轮是0时，旋转输入归零
    if (!input.keyboard_mode && (Cboard.GetScroll() == true || fabs(vw_handle) <= REMOTE_ROTATE_DEADZONE))
    {
        vw_handle = 0.0f;
    }

    // 限制输入，防止运动变形
    LimitChassisVector(vx_handle, vy_handle, vw_handle);

    // 跟随模式下，旋转输入由 PID 控制
    if (state == FOLLOW && !IsManualRotateRequested(input, vw_handle))
    {
        target.target_rotation = CalculateFollowRotation();
    }
    else    // 非跟随模式下，旋转输入由操作者直接控制
    {
        target.target_rotation = CHASSIS_ROTATION_GAIN * vw_handle;
    }

    /*
     * 保持原键鼠/遥控器输入轴映射：
     * 键鼠模式原逻辑传入 vy, vx；遥控器模式原逻辑传入 vx, vy。（遥控器坐标系定义和机器人坐标系x，y相反）
     */
    const float transform_vx = input.keyboard_mode ? vy_handle : vx_handle;
    const float transform_vy = input.keyboard_mode ? vx_handle : vy_handle;

    // 底盘坐标系下的平移目标
    float vx = 0.0f;
    float vy = 0.0f;
    // 计算旋转矩阵后的平移目标
    CalculateTranslationXY(Cboard.GetYawAngle(), transform_vx, transform_vy, 0.0f, &vx, &vy, 0.0f);

    // 斜坡规划
    omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx());
    omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy());
    
    // 获取斜坡规划后的目标
    target.target_translation_x = omni_target[0].GetOut();
    target.target_translation_y = omni_target[1].GetOut();

    return target;
}
