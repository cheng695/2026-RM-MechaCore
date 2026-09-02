#include "GimbalInputBuilder.hpp"

#include "Task/ControlTask.hpp"

namespace
{
constexpr float PI_VALUE = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI_VALUE / 180.0f;
} /* namespace */

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

GimbalInput BuildRuntimeGimbalInput()
{
    GimbalInput input = BuildGimbalInput(operator_input);

    /*
     * Planner 需要当前姿态做两件事：
     * 1. STOP 时锁住当前位置，避免重新使能后跳变；
     * 2. 折叠/起立过渡时判断动作是否已经到位。
     */
    input.current_yaw = HI12.GetAddYaw() * DEG_TO_RAD;
    input.current_yaw_angle = HI12.GetAngle(2) * DEG_TO_RAD;
    input.current_yaw_velocity = HI12.GetGyroRad(2);
    input.current_pitch = HI12.GetAngle(1) * DEG_TO_RAD;
    input.current_J1 = MotorJ4340.getAngleRad(1);
    input.current_J2 = MotorJ4310.getAngleRad(1);
    input.current_J1_velocity = MotorJ4340.getVelocityRads(1);
    input.current_J2_velocity = MotorJ4310.getVelocityRads(1);
    input.chassis_spin = Cboard.GetChassisSpin();

    /*
     * 鼠标、摇杆和视觉目标也在这里统一补齐。
     * 这样 GimbalController 只消费 GimbalInput，不再关心底层外设从哪里读取。
     */
    input.mouse_x = DT7.get_mouseX();
    input.mouse_y = DT7.get_mouseY();
    input.right_x = DT7.get_right_x();
    input.right_y = DT7.get_right_y();
    input.vision_yaw = vision.getTarYaw();
    input.vision_pitch = vision.getTarPitch();

    return input;
}
