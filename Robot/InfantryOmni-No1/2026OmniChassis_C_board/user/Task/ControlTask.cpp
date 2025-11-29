#include "ControlTask.hpp"

Alg::CalculationBase::Omni_IK omni_ik(1, 1);
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f)
};

void chassis_control()
{
    omni_ik.OmniInvKinematics(DT7.get_left_y(), DT7.get_left_x(), DT7.get_scroll_(), 0.0f, 8911.0f);
    for(int i = 0; i < 4; i++)
    {
        wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRpm(i+1));
    }
}


extern "C"{
void Control(void const * argument)
{
    for(;;)
    {
        chassis_control();
        osDelay(1);
    } 
}

}