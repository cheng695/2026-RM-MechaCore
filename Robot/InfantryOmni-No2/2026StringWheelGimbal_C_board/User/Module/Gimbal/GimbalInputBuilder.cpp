#include "GimbalInputBuilder.hpp"

GimbalInput BuildGimbalInput(const OperatorInput& operator_input)
{
    GimbalInput input;
    input.left_switch = operator_input.left_switch;
    input.right_switch = operator_input.right_switch;
    input.keyboard_mode = operator_input.keyboard_mode;
    input.mouse_right = operator_input.mouse_right;
    input.key_x = operator_input.key_x;
    input.key_v = operator_input.key_v;
    input.vision_enabled = operator_input.vision_enabled;
    input.transform_requested = operator_input.transform_requested;
    return input;
}
