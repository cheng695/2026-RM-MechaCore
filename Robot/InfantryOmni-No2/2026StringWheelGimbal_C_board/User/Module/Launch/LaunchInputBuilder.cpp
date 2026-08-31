#include "LaunchInputBuilder.hpp"

LaunchInput BuildLaunchInput(const OperatorInput& operator_input,
                             bool equipment_online,
                             bool jam_detected)
{
    LaunchInput input;
    input.left_switch = operator_input.left_switch;
    input.right_switch = operator_input.right_switch;
    input.equipment_online = equipment_online;
    input.keyboard_mode = operator_input.keyboard_mode;
    input.vision_enabled = operator_input.vision_enabled;
    input.launch_enabled = operator_input.launch_enabled;
    input.mouse_left = operator_input.mouse_left;
    input.mouse_left_pressed = operator_input.mouse_left_pressed;
    input.mouse_left_released = operator_input.mouse_left_released;
    input.fire_single = operator_input.fire_single;
    input.fire_continuous = operator_input.fire_continuous;
    input.jam_detected = jam_detected;
    return input;
}
