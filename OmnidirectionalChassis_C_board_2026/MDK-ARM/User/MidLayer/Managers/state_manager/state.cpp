#include "state.hpp"
#include "../User/LowLayer/Equipment/motor/djimotor.hpp"
#include "../User/LowLayer/Equipment/remote/dr16.hpp"

extern Buzzer::C_buzzer c_buzzer;

State::model chass_model;

void State::model::updateState() 
{
    // 这里可以安全地使用 dr16，因为在.cpp文件中包含了完整定义
    if (dr16.rc.s1 == 2 && dr16.rc.s2 == 2) 
    {
        current_state = STOP;
    }
    else if (dr16.rc.s1 == 2 && dr16.rc.s2 == 3) 
    {
        current_state = FOLLOW;
    }
    else if (dr16.rc.s1 == 3 && dr16.rc.s2 == 2) 
    {
        current_state = NOTFOLLOW;
    }
    else if(dr16.rc.s1 == 2 && dr16.rc.s2 == 1)
    {
        current_state = SHOT;
    }
    else if(dr16.rc.s1 == 1 && dr16.rc.s2 == 2)
    {
        current_state = VISION;
    }
    else if(dr16.rc.s1 == 1 && dr16.rc.s2 == 1)
    {
        current_state = KEYBOARD;
    }
    else 
    {
        current_state = STOP;
    }
}


void MotorState()
{
    extern motor::GM3508<4> Motor3508;

    // 调用对象的检查方法更新所有电机状态
    Motor3508.checkMotorsState();
    
    // 然后检查每个电机的在线状态
    for(int i = 0; i < Motor3508.GetMotorCount(); i++)
    {
        if(!Motor3508.IsMotorOnline(i))  // 使用公共方法检查在线状态
        {
            c_buzzer.Sound(i);// 处理第i个电机离线的情况
        }
    }
}

void RemoteState()
{
    extern Clicker::DR16 dr16;

    dr16.checkRemotecontrolState();

    if(!dr16.IsRemotecontrolOnline())
    {
        c_buzzer.remote();
    }
}
