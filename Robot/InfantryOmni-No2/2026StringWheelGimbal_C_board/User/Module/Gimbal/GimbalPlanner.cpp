#include "GimbalPlanner.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;

/*
 * 云台关节角度约定。
 *
 * 达妙 MIT 位置控制接口使用弧度，因此这里所有角度目标都用 rad。
 * J1 是主 Pitch 关节，正常起立位置约为 103 度。
 * J2 是末端 Pitch 关节，0 rad 表示让末端保持世界坐标系水平。
 */
constexpr float NORMAL_J1_ANGLE = 103.0f * DEG_TO_RAD;
constexpr float NORMAL_J2_ANGLE = 0.0f;

/*
 * 折叠状态目标角度。
 *
 * DOWN_YAW_ANGLE 是折叠前需要先对齐的 yaw 固定角。
 * DOWN_J1_ANGLE 是 J1 折叠后的目标角，当前按 153 度作为初值。
 */
constexpr float DOWN_YAW_ANGLE = 0.0f;
constexpr float DOWN_J1_ANGLE = 153.0f * DEG_TO_RAD;

/* Yaw 接近折叠固定角后，才允许 J1 开始折叠。 */
constexpr float DOWN_YAW_READY_TOLERANCE = 3.0f * DEG_TO_RAD;

/*
 * 折叠/起立过程的关节最大规划速度，单位 rad/s。
 * 这里限制的是“目标值变化速度”，不是电机反馈速度；它能避免 J1/J2 目标一步跳变。
 */
constexpr float J1_TRANSITION_MAX_SPEED = 0.8f;
constexpr float J2_TRANSITION_MAX_SPEED = 0.8f;

/* 只有 J1 已经处于正常起立姿态时，J2 才使用这组活动范围。 */
constexpr float J2_UPRIGHT_MIN = -0.3491f;
constexpr float J2_UPRIGHT_MAX = 0.5236f;

/* Planner 当前由 200 Hz 控制循环调用，规划间隔为 5 ms。 */
constexpr float CONTROL_PERIOD = 0.005f;

/*
 * 鼠标 yaw 输入增益。
 */
constexpr float MOUSE_YAW_SENSITIVITY = 0.3f;

/*
 * 鼠标 J2 输入增益。
 */
constexpr float MOUSE_PITCH_SENSITIVITY = 0.004f;

/*
 * 遥控器右摇杆 yaw 输入增益。
 */
constexpr float STICK_YAW_SENSITIVITY = 0.01f;

/*
 * 遥控器右摇杆 J2 输入增益。
 */
constexpr float STICK_PITCH_SENSITIVITY = 0.007f;

/* 判断是否正在执行折叠流程。 */
bool IsDownTransition(GimbalTransition transition)
{
    return transition == GimbalTransition::StopToDown ||
           transition == GimbalTransition::ManualToDown ||
           transition == GimbalTransition::VisionToDown;
}

/* 判断是否正在执行起立流程。 */
bool IsUprightTransition(GimbalTransition transition)
{
    return transition == GimbalTransition::StopToManual ||
           transition == GimbalTransition::DownToManual;
}

float NormalizeAngle(float angle)
{
    while (angle > PI)
    {
        angle -= 2.0f * PI;
    }

    while (angle < -PI)
    {
        angle += 2.0f * PI;
    }

    return angle;
}

float StepToTarget(float current, float target, float max_step)
{
    const float error = target - current;

    if (error > max_step)
    {
        return current + max_step;
    }

    if (error < -max_step)
    {
        return current - max_step;
    }

    return target;
}
} // namespace

void InitGimbalPlanner()
{
    /* 当前目标参数为编译期常量，预留运行时参数初始化入口。 */
}

GimbalTarget PlanGimbalTarget(Enum_Gimbal_States state,
                              GimbalTransition transition,
                              const GimbalInput& input,
                              const GimbalTarget& current_target)
{
    GimbalTarget target = current_target;

    /*
     * 折叠流程优先级最高。
     *
     * 顺序是：Yaw 先转到固定折叠对齐角，Yaw 到位后 J1 再进入折叠角。
     * J2 在折叠全过程中固定为 0 rad，让末端保持世界坐标系水平。
     * 折叠状态不使用 J2 起立姿态下的限幅，因为机械可动范围不同。
     */
    if (IsDownTransition(transition))
    {
        target.target_yaw = DOWN_YAW_ANGLE;
        target.target_J2 = StepToTarget(current_target.target_J2,
                                        NORMAL_J2_ANGLE,
                                        J2_TRANSITION_MAX_SPEED * CONTROL_PERIOD);

        if (std::fabs(NormalizeAngle(input.current_yaw - DOWN_YAW_ANGLE)) <= DOWN_YAW_READY_TOLERANCE)
        {
            target.target_J1 = StepToTarget(current_target.target_J1,
                                            DOWN_J1_ANGLE,
                                            J1_TRANSITION_MAX_SPEED * CONTROL_PERIOD);
        }
        else
        {
            target.target_J1 = current_target.target_J1;
        }

        return target;
    }

    /*
     * 起立流程同样优先于普通 MANUAL/VISION 控制。
     *
     * 从 DOWN 或 STOP 起立时，Yaw 保持当前角度，不主动转向；
     * J1 回到正常起立位置；J2 固定为 0 rad，保持世界坐标系水平。
     * 由于 J1 尚未到位，这里不能使用起立姿态下的 J2 限幅。
     */
    if (IsUprightTransition(transition))
    {
        target.target_yaw = input.current_yaw;
        target.target_J1 = StepToTarget(current_target.target_J1,
                                        NORMAL_J1_ANGLE,
                                        J1_TRANSITION_MAX_SPEED * CONTROL_PERIOD);
        target.target_J2 = StepToTarget(current_target.target_J2,
                                        NORMAL_J2_ANGLE,
                                        J2_TRANSITION_MAX_SPEED * CONTROL_PERIOD);
        return target;
    }

    switch (state)
    {
    case STOP:
        /* 停止状态锁住三个关节的当前位置，避免重新使能时产生目标跳变。 */
        target.target_yaw = input.current_yaw;
        target.target_J1 = input.current_J1;
        target.target_J2 = input.current_J2;
        break;

    case VISION:
        /* 视觉模式只控制 Yaw 和 J2，J1 始终保持正常起立位置。 */
        target.target_yaw = -input.vision_yaw * DEG_TO_RAD;
        target.target_J1 = NORMAL_J1_ANGLE;
        target.target_J2 = input.vision_pitch * DEG_TO_RAD;
        target.target_J2 = std::clamp(target.target_J2,
                                      J2_UPRIGHT_MIN, J2_UPRIGHT_MAX);
        break;

    case MANUAL:
        /* 手动模式不允许操作员直接改变 J1，J1 始终保持正常起立位置。 */
        target.target_J1 = NORMAL_J1_ANGLE;

        if (input.left_switch == 3 && input.right_switch == 3)
        {
            /* 键鼠模式：鼠标输入视为角速度，再积分成角度目标。 */
            target.target_yaw += MOUSE_YAW_SENSITIVITY *
                                 -input.mouse_x * CONTROL_PERIOD;

            float mouse_y = input.mouse_y;
            /* 丢弃异常脉冲和鼠标死区，避免目标角度突然跳变。 */
            if (mouse_y > 200.0f || mouse_y < -200.0f ||
                (mouse_y < 1.0f && mouse_y > -1.0f))
            {
                mouse_y = 0.0f;
            }

            target.target_J2 += MOUSE_PITCH_SENSITIVITY *
                                mouse_y * CONTROL_PERIOD;
        }
        else
        {
            /* 遥控器模式：右摇杆输入同样转成角度目标增量。 */
            target.target_yaw += STICK_YAW_SENSITIVITY *
                                 -input.right_x * CONTROL_PERIOD;
            target.target_J2 += STICK_PITCH_SENSITIVITY *
                                input.right_y * CONTROL_PERIOD;
        }

        /* 只有起立完成后的普通操作，才使用 J2 起立姿态活动范围。 */
        target.target_J2 = std::clamp(target.target_J2,
                                      J2_UPRIGHT_MIN, J2_UPRIGHT_MAX);
        break;

    case DOWN:
        /* 折叠完成后保持固定 Yaw、J1，并让 J2 保持世界坐标系水平。 */
        target.target_yaw = DOWN_YAW_ANGLE;
        target.target_J1 = DOWN_J1_ANGLE;
        target.target_J2 = NORMAL_J2_ANGLE;
        break;

    default:
        /* 未知状态采用当前位置作为安全目标。 */
        target.target_yaw = input.current_yaw;
        target.target_J1 = input.current_J1;
        target.target_J2 = input.current_J2;
        break;
    }

    return target;
}
