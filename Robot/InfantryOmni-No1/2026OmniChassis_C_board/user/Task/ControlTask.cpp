#include "ControlTask.hpp"

Class_FSM chassis_fsm;   
Alg::CalculationBase::Omni_IK omni_ik(1, 1);
ALG::PID::PID YawAngle_pid(0.0f, 0.0f, 0.0f, 25000.0f, 2500.0f, 200.0f);
ALG::PID::PID YawVelocity_pid(0.0f, 0.0f, 0.0f, 25000.0f, 2500.0f, 200.0f);
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f)
};

void chassis_fsm_init()
{
    chassis_fsm.Init();
}

bool check_online()
{
    // for(int i = 0; i < 4; i++)
    // {
    //     if(!Motor3508.isConnected(i+1))
    //     {
    //         return false;
    //     }
    // }

    if(!DT7.isConnected() || !Motor6020.isConnected(1))
    {
        return false;
    }
    
    return true;
}

void main_loop(uint8_t left_sw, uint8_t right_sw, bool is_online) 
{   
    chassis_fsm.StateUpdate(left_sw, right_sw, is_online);

    switch(chassis_fsm.Get_Now_State()) 
    {
        case STOP:
            chassis_stop();
            break;
        case FOLLOW:
            //chassis_follow();
            break;
        case NOTFOLLOW:
            chassis_not_follow();
            break;
        case KEYBOARD:
            //chassis_keyboard_control();
            break;
        default:
            chassis_stop();
            break;
    }
}

void chassis_stop()
{
    for(int i = 0; i < 4; i++)
    {
        wheel_pid[i].reset();
        YawVelocity_pid.reset();
        YawAngle_pid.reset();
    }
}

float YawTarget = 0.0f;
void chassis_not_follow()
{
    // omni_ik.OmniInvKinematics(DT7.get_left_y(), DT7.get_left_x(), DT7.get_scroll_(), 0.0f, 8911.0f, 8911.0f);
    // for(int i = 0; i < 4; i++)
    // {
    //     wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRpm(i+1));
    // }
    YawTarget += 0.1f*DT7.get_left_x();
    YawAngle_pid.UpDate(YawTarget, Motor6020.getAddAngleDeg(1));
    YawVelocity_pid.UpDate(YawAngle_pid.getOutput(), Motor6020.getVelocityRpm(1));
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
        
        main_loop(DT7.get_s1(), DT7.get_s2(), check_online());

        osDelay(1);
    } 
}
}