#include "Control.hpp"

Control::motor_control wheel_control;
Control::remote_control remote_control;

void Control::motor_control::wheelControl()
{
    model_choose.ModeSelection();
    chassis_pidtarget.Setpidtarget();
    motor_send.SendToMotor();
}

void Control::remote_control::remoteControl()
{
    dr16_tar.TargetDataUpdate();
}
