#include "GimbalPlanner.hpp"

#include <algorithm>

namespace
{
constexpr float PI = 3.14159265358979323846f;

/*
 * 云台关节角度约定：
 * J1 为主 Pitch 关节，控制坐标 103 度正常起立位置；
 * J2 为辅助 Pitch 关节，控制坐标 0 弧度 对应世界坐标系水平姿态。
 */
constexpr float NORMAL_J1_ANGLE = 103.0f;
constexpr float NORMAL_J2_ANGLE = 0.0f;

/* 折叠状态的标定目标，单位为弧度（yaw）度（J1），需根据实际机械位置修改。 */
constexpr float DOWN_YAW_ANGLE = 0.0f;
constexpr float DOWN_J1_ANGLE = 153.0f;

/* 只有 J1 已经处于起立姿态时，才允许 J2（弧度） 使用这组活动范围。 */
constexpr float J2_UPRIGHT_MIN = -0.3491f;
constexpr float J2_UPRIGHT_MAX = 0.5236f;

/* Planner 当前由 200 Hz 控制循环调用，即每次规划间隔 5 ms。 */
constexpr float CONTROL_PERIOD = 0.005f;

/*
 * 鼠标输入增益：将鼠标每帧的位移量换算为 yaw 目标角度增量。
 * 计算形式为：mouse_x * MOUSE_YAW_SENSITIVITY * CONTROL_PERIOD。
 * 该值越大，鼠标横向移动相同距离时 yaw 转动越快。
 */
constexpr float MOUSE_YAW_SENSITIVITY = 0.3f;

/*
 * 鼠标输入增益：将鼠标每帧的纵向位移量换算为 J2 目标角度增量。
 * 该值越大，鼠标纵向移动相同距离时 J2 变化越快。
 */
constexpr float MOUSE_PITCH_SENSITIVITY = 0.004f;

/*
 * 右摇杆输入增益：将摇杆横向输入换算为 yaw 目标角度增量。
 * 该值越大，右摇杆横向偏转相同幅度时 yaw 转动越快。
 */
constexpr float STICK_YAW_SENSITIVITY = 0.01f;

/*
 * 右摇杆输入增益：将摇杆纵向输入换算为 J2 目标角度增量。
 * 该值越大，右摇杆纵向偏转相同幅度时 J2 变化越快。
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
     * 折叠过程优先级最高。
     * Yaw 必须转到固定的折叠对齐角，J1 转到折叠角，J2 始终保持水平。
     * 这里不对 J2 做折叠限幅，因为折叠状态的活动范围与起立状态不同。
     */
    if (IsDownTransition(transition))
    {
        target.target_yaw = DOWN_YAW_ANGLE;
        target.target_J1 = DOWN_J1_ANGLE;
        target.target_J2 = NORMAL_J2_ANGLE;
        return target;
    }

    /*
     * 起立过程同样优先于普通 MANUAL/VISION 控制。
     * Yaw 保持进入起立流程时的当前角度，避免起立时云台突然转向；
     * J1 回到正常位置，J2 固定为 0 弧度以保持世界坐标系水平。
     * 此时 J1 尚未完全到位，因此不能使用起立状态下的 J2 限幅。
     */
    if (IsUprightTransition(transition))
    {
        target.target_yaw = input.current_yaw;
        target.target_J1 = NORMAL_J1_ANGLE;
        target.target_J2 = NORMAL_J2_ANGLE;
        return target;
    }

    switch (state)
    {
    case STOP:
        /* 急停状态锁定三个关节的当前位置，避免重新使能时产生跳变。 */
        target.target_yaw = input.current_yaw;
        target.target_J1 = input.current_J1;
        target.target_J2 = input.current_J2;
        break;

    case VISION:
        /* 视觉只控制 Yaw 和 J2，J1 保持在正常起立位置。 */
        target.target_yaw = -input.vision_yaw * PI / 180.0f;
        target.target_J1 = NORMAL_J1_ANGLE;
        target.target_J2 = input.vision_pitch * PI / 180.0f;
        target.target_J2 = std::clamp(target.target_J2,
                                      J2_UPRIGHT_MIN, J2_UPRIGHT_MAX);
        break;

    case MANUAL:
        /* 普通手动模式不允许 J1 被操作员输入改变。 */
        target.target_J1 = NORMAL_J1_ANGLE;

        if (input.left_switch == 3 && input.right_switch == 3)
        {
            /* 键鼠模式：输入被视为角速度，积分为角度目标增量。 */
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
            /* 遥控器模式：右摇杆输入同样转换为角度目标增量。 */
            target.target_yaw += STICK_YAW_SENSITIVITY *
                                 -input.right_x * CONTROL_PERIOD;
            target.target_J2 += STICK_PITCH_SENSITIVITY *
                                input.right_y * CONTROL_PERIOD;
        }

        /* 起立完成后的正常操作才使用 J2 的活动范围。 */
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
