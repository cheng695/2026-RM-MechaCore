#include "ChassisEventDetector.hpp"
#include "DT7.hpp"

namespace
{
const uint8_t UP = BSP::REMOTE_CONTROL::RemoteController::UP;
const uint8_t MIDDLE = BSP::REMOTE_CONTROL::RemoteController::MIDDLE;
const uint8_t DOWN = BSP::REMOTE_CONTROL::RemoteController::DOWN;
} /* namespace */

/*
 * 根据当前输入快照解析底盘状态机事件。
 *
 * 遥控器三挡约定：
 * UP     = 上挡；
 * MIDDLE = 中挡；
 * DOWN   = 下挡。
 *
 * 这里不直接修改 FSM 状态，只把“当前挡位应该请求进入什么状态”
 * 翻译成 StopRequested / FollowRequested / NotFollowRequested。
 */
ChassisEvent ResolveChassisEvent(const OperatorInput& input, bool equipment_online)
{
    /*
     * 设备离线是最高优先级安全条件。
     * 任意挡位下，只要关键设备离线，底盘都请求进入 STOP。
     */
    if (!equipment_online)
    {
        return ChassisEvent::StopRequested;
    }

    /*
     * 左上：
     * 右上 / 右中 / 右下全部为底盘不跟随。
     */
    if (input.left_switch == UP)
    {
        if (input.right_switch == UP ||
            input.right_switch == MIDDLE ||
            input.right_switch == DOWN)
        {
            return ChassisEvent::NotFollowRequested;
        }
    }

    /*
     * 左中：
     * 左中右上：底盘跟随；
     * 左中右中：键鼠模式，由 Q 键 toggle 决定跟随/不跟随；
     * 左中右下：底盘不跟随。
     */
    if (input.left_switch == MIDDLE)
    {
        if (input.right_switch == UP)
        {
            return ChassisEvent::FollowRequested;
        }

        if (input.right_switch == MIDDLE)
        {
            return input.follow_enabled ? ChassisEvent::FollowRequested : ChassisEvent::NotFollowRequested;
        }

        if (input.right_switch == DOWN)
        {
            return ChassisEvent::NotFollowRequested;
        }
    }

    /*
     * 左下：
     * 左下右上：底盘跟随；
     * 左下右中：底盘不跟随；
     * 左下右下：底盘停止。
     */
    if (input.left_switch == DOWN)
    {
        if (input.right_switch == UP)
        {
            return ChassisEvent::FollowRequested;
        }

        if (input.right_switch == MIDDLE)
        {
            return ChassisEvent::NotFollowRequested;
        }

        if (input.right_switch == DOWN)
        {
            return ChassisEvent::StopRequested;
        }
    }

    /*
     * 未知挡位组合视为异常输入，默认请求 STOP。
     */
    return ChassisEvent::StopRequested;
}

/*
 * 检测是否需要向底盘 FSM 派发事件。
 *
 * 输入没有变化时返回 None，避免每个控制周期重复派发同一个状态请求；
 * 输入变化包括：设备在线状态、左右拨杆状态、键鼠模式下的跟随开关。
 */
ChassisEvent DetectChassisEvent(const OperatorInput& input, bool equipment_online)
{
    static uint8_t last_left = DOWN;
    static uint8_t last_right = DOWN;
    static bool last_online = false;
    static bool last_follow_enabled = true;

    const bool changed =
        (equipment_online != last_online) ||
        (input.left_switch != last_left) ||
        (input.right_switch != last_right) ||
        (input.follow_enabled != last_follow_enabled);

    last_left = input.left_switch;
    last_right = input.right_switch;
    last_online = equipment_online;
    last_follow_enabled = input.follow_enabled;

    if (!changed)
    {
        return ChassisEvent::None;
    }

    return ResolveChassisEvent(input, equipment_online);
}
