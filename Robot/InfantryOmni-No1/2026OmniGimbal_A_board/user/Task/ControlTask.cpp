#include "ControlTask.hpp"

ALG::ADRC::FirstLADRC yaw_adrc(22.0f, 88.0f, 0.065f, 0.005f, 25000.0f);
ALG::PID::PID pitch_AnglePid(45.0f, 0.0f, 0.0f, 12.56f, 2500.0f, 200.0f);
ALG::PID::PID pitch_VelocityPid(0.75f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);
ALG::PID::PID yaw_pid(5.0f, 0.0f, 0.0f, 25000.0f, 2500.0f, 200.0f);
ALG::PID::PID dial_pid(1.0f, 0.0f, 0.0f, 25000.0f, 2500.0f, 200.0f);
ALG::PID::PID surgewheel_pid[2] = {
    ALG::PID::PID(1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f),
    ALG::PID::PID(1.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 200.0f)
};

// void chassis_fsm_init()
// {
//     chassis_fsm.Init();
// }

// bool check_online()
// {
//     for(int i = 0; i < 2; i++)
//     {
//         if(!Motor3508.isConnected(i+1))
//         {
//             return false;
//         }
//     }

//     if(!DT7.isConnected() || !HI12.isConnected() || !Motor6020.isConnected() || 
//         !MotorJ4310.isConnected() || !Motor2006.isConnected())
//     {
//         return false;
//     }
    
//     return true;
// }

// void main_loop(uint8_t left_sw, uint8_t right_sw, bool is_online) 
// {   
//     chassis_fsm.StateUpdate(left_sw, right_sw, is_online);

//     switch(chassis_fsm.Get_Now_State()) 
//     {
//         case STOP:
//             chassis_stop();
//             break;
//         case FOLLOW:
//             //chassis_follow();
//             break;
//         case NOTFOLLOW:
//             chassis_not_follow();
//             break;
//         case KEYBOARD:
//             //chassis_keyboard_control();
//             break;
//         default:
//             chassis_stop();
//             break;
//     }
// }

// void chassis_stop()
// {
//     for(int i = 0; i < 4; i++)
//     {
//         wheel_pid[i].reset();
//     }
// }

// void chassis_not_follow()
// {
//     omni_ik.OmniInvKinematics(DT7.get_left_y(), DT7.get_left_x(), DT7.get_scroll_(), 0.0f, 8911.0f, 8911.0f);
//     for(int i = 0; i < 4; i++)
//     {
//         wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRpm(i+1));
//     }
// }

float target_yaw;
extern "C"{
void Control(void const * argument)
{
    // // 初始化蜂鸣器管理器
    // BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    
    // chassis_fsm_init();
    for(;;)
    {
        // // 更新蜂鸣器管理器，处理队列中的响铃请求
        // BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        // main_loop(DT7.get_s1(), DT7.get_s2(), check_online());
        target_yaw += DT7.get_left_x();
        yaw_pid.UpDate(target_yaw, Motor6020.getAddAngleDeg(1));

        osDelay(5);
    } 
}
}