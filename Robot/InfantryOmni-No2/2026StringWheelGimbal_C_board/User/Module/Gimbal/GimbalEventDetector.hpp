#pragma once

#include <cstdint>

#include "GimbalStateMachine.hpp"

/*
 * 将遥控器、视觉和设备在线状态转换为云台状态机事件。
 * 状态机只接收 GimbalEvent，不直接读取这些原始输入。
 */
GimbalEvent ResolveGimbalEvent(uint8_t left_switch,
                               uint8_t right_switch,
                               bool equipment_online,
                               bool vision_enabled,
                               bool transform_requested = false);

GimbalEvent DetectGimbalEvent(uint8_t left_switch,
                              uint8_t right_switch,
                              bool equipment_online,
                              bool vision_enabled,
                              bool transform_requested = false);
