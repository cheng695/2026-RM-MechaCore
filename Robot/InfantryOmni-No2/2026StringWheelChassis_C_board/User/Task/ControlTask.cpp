#include "ControlTask.hpp"
#define PI_ 3.1415926535897932384626433832795

Chassis_FSM chassis_fsm;
float wheel_azimuth[4] = {7*PI_/4, PI_/4, 3*PI_/4, 5*PI_/4};
float phase_offset[4] = {-1.3f, 3.4f, 2.8f, 1.4f};
Alg::CalculationBase::String_IK string_ik(1.175f, 0.055f, wheel_azimuth, phase_offset);

ALG::PID::PID stringAngle_pid[4] = {
    ALG::PID::PID(3.0f, 0.002f, 0.0f, 16384.0f, 2500.0f, 5.0f),
    ALG::PID::PID(3.0f, 0.002f, 0.0f, 16384.0f, 2500.0f, 5.0f),
    ALG::PID::PID(3.0f, 0.002f, 0.0f, 16384.0f, 2500.0f, 5.0f),
    ALG::PID::PID(3.0f, 0.002f, 0.0f, 16384.0f, 2500.0f, 5.0f)
};
ALG::PID::PID stringVelocity_pid[4] = {
    ALG::PID::PID(50.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(50.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(50.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),
    ALG::PID::PID(50.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f)
};
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(3.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f),
    ALG::PID::PID(3.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f),
    ALG::PID::PID(3.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f),
    ALG::PID::PID(3.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f)
};

ControlTask chassis_target;
Output_chassis chassis_output;

void chassis_fsm_init()
{
    chassis_fsm.Init();
}

bool check_online()
{
    bool isconnected = true;
    for(int i = 0; i < 4; i++)
    {
        if(!Motor3508.isConnected(i+1, i+1) || !Motor6020.isConnected(i+1, i+5))
        {
            isconnected = false;
        }
    }

    if(!DT7.isConnected())
    {
        isconnected = false;
    }
    
    if(!isconnected)
    {
        return false;
    }

    return true;
}

void Settarget_chassis()
{
    chassis_target.target_translation_x = DT7.get_left_y();
    chassis_target.target_translation_y = DT7.get_left_x();
    chassis_target.target_rotation = DT7.get_scroll_();
}

void chassis_stop()
{
    for(int i = 0; i < 4; i++)
    {
        wheel_pid[i].reset();
        stringAngle_pid[i].reset();
        stringVelocity_pid[i].reset();
        chassis_output.out_string[i] = 0.0f;
        chassis_output.out_wheel[i] = 0.0f;
    }
}


float motor_direction[4];
void chassis_notfollow()
{
    for(int i = 0; i < 4; i++)
    {
        string_ik.Set_current_steer_angles(Motor6020.getAngleRad(i+1), i);
    }
    string_ik.StringInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 1.0f, 1.0f);
    
    for(int i = 0; i < 4; i++)
    {
        motor_direction[i] = string_ik.GetMotor_direction(i) * 57.3f; //弧度转角度
        float motor_angle = Motor6020.getAngleDeg(i+1); //反馈角度

        stringAngle_pid[i].UpDate(motor_direction[i], motor_angle);
        stringVelocity_pid[i].UpDate(stringAngle_pid[i].getOutput(), Motor6020.getVelocityRpm(i+1));

        wheel_pid[i].UpDate(string_ik.GetMotor_wheel(i), Motor3508.getVelocityRpm(i+1));

        chassis_output.out_string[i] = stringVelocity_pid[i].getOutput();
        chassis_output.out_wheel[i] = wheel_pid[i].getOutput();
    }
    
}

void chassis_follow()
{

}

void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online) 
{   
    chassis_fsm.StateUpdate(left_sw, right_sw, is_online);
    Settarget_chassis();

    switch(chassis_fsm.Get_Now_State()) 
    {
        case STOP:
            chassis_stop();
            break;
        case FOLLOW:
            chassis_follow();
            break;
        case NOTFOLLOW:
            chassis_notfollow();
            break;
        case KEYBOARD:
            //chassis_keyboard_control();
            break;
        default:
            chassis_stop();
            break;
    }
}

extern "C"{
void Control(void const * argument)
{
    // 初始化蜂鸣器管理器
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    chassis_fsm_init();
    for(;;)
    {
        // 更新蜂鸣器管理器，处理队列中的响铃请求
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online());

        osDelay(1);
    } 
}
}