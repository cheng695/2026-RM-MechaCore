#include "GimbalEventDetector.hpp"
#include "DT7.hpp"

namespace
{
/* 使用 SWITCH_ 前缀，避免与云台的 DOWN 状态名称冲突。 */
const uint8_t SWITCH_UP = BSP::REMOTE_CONTROL::RemoteController::UP;
const uint8_t SWITCH_MIDDLE = BSP::REMOTE_CONTROL::RemoteController::MIDDLE;
const uint8_t SWITCH_DOWN = BSP::REMOTE_CONTROL::RemoteController::DOWN;
} /* namespace */

/*
 * 根据当前输入解析云台的目标模式。
 *
 * 这个函数只负责“输入 -> 事件”的转换，不直接修改云台状态，也不执行电机动作。
 * Yaw 对齐、Pitch 回正和 Pitch 折叠等动作由 Gimbal_FSM 和控制层负责。
 */
GimbalEvent ResolveGimbalEvent(uint8_t left_switch,
                               uint8_t right_switch,
                               bool equipment_online,
                               bool vision_enabled,
                               bool transform_requested)
{
    /*
     * 设备离线是最高优先级的安全条件。
     * 无论当前拨杆处于什么位置，只要关键设备离线，就请求进入 STOP。
     */
    if (!equipment_online)
    {
        return GimbalEvent::StopRequested;
    }

    /*
     * 左上挡：
     *   右上 -> DOWN，执行云台折叠；
     *   右中 -> 视觉标志有效时进入 VISION，否则回到 MANUAL；
     *   右下 -> MANUAL。
     *
     * 左上 + 右上明确表示 DOWN。
     */
    if (left_switch == SWITCH_UP)
    {
        if (right_switch == SWITCH_UP)
        {
            return GimbalEvent::DownRequested;
        }

        if (right_switch == SWITCH_MIDDLE)
        {
            return vision_enabled ? GimbalEvent::VisionRequested
                                  : GimbalEvent::ManualRequested;
        }

        if (right_switch == SWITCH_DOWN)
        {
            return GimbalEvent::ManualRequested;
        }
    }

    /*
     * 左中挡：
     *   右上 -> MANUAL；
     *   右中 -> 键鼠模式。
     *
     * 键鼠模式下，transform_requested 表示折叠/变形请求，
     * vision_enabled 表示视觉请求。折叠动作具有更高优先级，
     * 因为进入 DOWN 后不能直接切换到 VISION。
     *   右下 -> MANUAL。
     */
    if (left_switch == SWITCH_MIDDLE)
    {
        if (right_switch == SWITCH_UP)
        {
            return GimbalEvent::ManualRequested;
        }

        if (right_switch == SWITCH_MIDDLE)
        {
            if (transform_requested)
            {
                return GimbalEvent::DownRequested;
            }

            return vision_enabled ? GimbalEvent::VisionRequested
                                  : GimbalEvent::ManualRequested;
        }

        if (right_switch == SWITCH_DOWN)
        {
            return GimbalEvent::ManualRequested;
        }
    }

    /*
     * 左下挡：
     *   右下 -> STOP；
     *   右上或右中 -> MANUAL。
     */
    if (left_switch == SWITCH_DOWN)
    {
        if (right_switch == SWITCH_DOWN)
        {
            return GimbalEvent::StopRequested;
        }

        if (right_switch == SWITCH_UP || right_switch == SWITCH_MIDDLE)
        {
            return GimbalEvent::ManualRequested;
        }
    }

    /* 未知挡位按异常输入处理，默认请求安全停止。 */
    return GimbalEvent::StopRequested;
}

/*
 * 检测目标事件是否发生变化。
 *
 * ResolveGimbalEvent() 每次都会给出当前输入对应的目标事件，
 * 而本函数只在目标事件变化时向状态机发送一次，避免控制周期内重复派发。
 */
GimbalEvent DetectGimbalEvent(uint8_t left_switch,
                              uint8_t right_switch,
                              bool equipment_online,
                              bool vision_enabled,
                              bool transform_requested)
{
    static GimbalEvent last_event = GimbalEvent::None;

    const GimbalEvent current_event = ResolveGimbalEvent(left_switch,
                                                         right_switch,
                                                         equipment_online,
                                                         vision_enabled,
                                                         transform_requested);

    if (current_event == last_event)
    {
        return GimbalEvent::None;
    }

    last_event = current_event;
    return current_event;
}
