#include "LaunchEventDetector.hpp"
#include "DT7.hpp"

namespace
{
const uint8_t SWITCH_UP = BSP::REMOTE_CONTROL::RemoteController::UP;
const uint8_t SWITCH_MIDDLE = BSP::REMOTE_CONTROL::RemoteController::MIDDLE;
const uint8_t SWITCH_DOWN = BSP::REMOTE_CONTROL::RemoteController::DOWN;

/* 将视觉端或键鼠端的单发/连发请求转换为发射模式事件。 */
LaunchEvent ResolveFireMode(const LaunchInput& input)
{
    /* 两个模式同时有效属于冲突输入，默认停火。 */
    if (input.fire_single && input.fire_continuous)
    {
        return LaunchEvent::CeasefireRequested;
    }

    if (input.fire_continuous)
    {
        return LaunchEvent::AutoRequested;
    }

    if (input.fire_single)
    {
        return LaunchEvent::OnlyRequested;
    }

    return LaunchEvent::CeasefireRequested;
}
}

LaunchEvent ResolveLaunchEvent(const LaunchInput& input,
                               Enum_Launch_States current_state)
{
    /* 设备离线是最高优先级安全事件。 */
    if (!input.equipment_online)
    {
        return LaunchEvent::StopRequested;
    }

    /* 左下，或左上右下，表示发射机构完全停止。 */
    if (input.left_switch == SWITCH_DOWN ||
        (!input.keyboard_mode && input.left_switch == SWITCH_UP && input.right_switch == SWITCH_DOWN))
    {
        return LaunchEvent::StopRequested;
    }

    /* STOP 优先于 JAM，但 JAM 优先于普通模式请求。 */
    if (input.jam_detected)
    {
        return LaunchEvent::JamDetected;
    }

    if (input.left_switch == SWITCH_UP)
    {
        /* 左上右上对应云台 DOWN，发射机构只允许手动连发。 */
        if (input.right_switch == SWITCH_UP)
        {
            return LaunchEvent::AutoRequested;
        }

        /* 左上右中由视觉端决定停火、单发或连发。 */
        if (input.right_switch == SWITCH_MIDDLE)
        {
            return input.vision_enabled
                ? ResolveFireMode(input)
                : LaunchEvent::CeasefireRequested;
        }

        return LaunchEvent::CeasefireRequested;
    }

    if (input.left_switch == SWITCH_MIDDLE)
    {
        /* 左中右上：遥控器连发。 */
        if (input.right_switch == SWITCH_UP)
        {
            return LaunchEvent::AutoRequested;
        }

        /* 左中右下：遥控器单发。 */
        if (input.right_switch == SWITCH_DOWN)
        {
            return LaunchEvent::OnlyRequested;
        }

        /* 左中右中：键鼠模式，必须先由 B 键使能发射。 */
        if (input.right_switch == SWITCH_MIDDLE)
        {
            if (!input.launch_enabled)
            {
                return LaunchEvent::StopRequested;
            }

            if (input.vision_enabled)
            {
                return ResolveFireMode(input);
            }

            /*
             * 键鼠模式下，鼠标左键是连发入口；Z 选择单发，X 选择连发。
             * 鼠标左键优先于 Z，避免同时操作时出现模式冲突。
             */
            if (input.mouse_left_pressed)
            {
                /* ONLY 模式下，左键此前释放过则继续保持单发模式。 */
                if (current_state == LAUNCH_ONLY && input.mouse_left_released)
                {
                    return LaunchEvent::OnlyRequested;
                }

                return LaunchEvent::AutoRequested;
            }

            return ResolveFireMode(input);
        }
    }

    /* 未知挡位默认停止，避免异常输入导致发射。 */
    return LaunchEvent::StopRequested;
}

LaunchEvent DetectLaunchEvent(const LaunchInput& input,
                              Enum_Launch_States current_state)
{
    static LaunchEvent last_event = LaunchEvent::None;

    const LaunchEvent current_event = ResolveLaunchEvent(input, current_state);
    if (current_event == last_event)
    {
        return LaunchEvent::None;
    }

    last_event = current_event;
    return current_event;
}
