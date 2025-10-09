#include "state.hpp"
#include "../User/LowLayer/Equipment/motor/djimotor.hpp"
#include "../User/LowLayer/Equipment/remote/dr16.hpp"
#include "../User/MidLayer/Managers/comm_manager/CtoA.hpp"

extern Buzzer::C_buzzer c_buzzer;
extern motor::GM3508<4> Motor3508;
extern Clicker::DR16 dr16;
extern BoardComm::CtoA Cboard;

State::model chass_model;

void State::model::updateState() 
{
    static const model_state state_choose[4][4] = {
        {STOP, STOP          , STOP       , STOP},
        {STOP, KEYBOARD      , SHOT_FOLLOW, SHOT_Rotate},
        {STOP, VISION_NOTSHOT, STOP       , NOTFOLLOW},
        {STOP, VISION_SHOT   , FOLLOW     , STOP}
    };

    if(dr16.rc.s1 >= 1 && dr16.rc.s1 <= 3 && dr16.rc.s2 >= 1 && dr16.rc.s2 <= 3)
    {
        current_state = state_choose[dr16.rc.s1][dr16.rc.s2];
    }
    else
    {
        current_state = STOP;
    }

    if(!dr16.GetIsRemotecontrolOnline())
    {
        current_state = STOP;
    }

    for(int i = 0; i < Motor3508.GetMotorCount(); i++)
    {
        if(!Motor3508.GetIsMotorOnline(i))  // 使用公共方法检查在线状态
        {
            current_state = STOP;
        }
    }

    if(current_state != last_state)
    {
        is_statechanged = true;
        last_state = current_state;
    }
    else
    {
        is_statechanged = false;
    }
}

void MotorState()
{
    // 调用对象的检查方法更新所有电机状态
    Motor3508.checkMotorsState();
    
    // 然后检查每个电机的在线状态
    for(int i = 0; i < Motor3508.GetMotorCount(); i++)
    {
        if(!Motor3508.GetIsMotorOnline(i))  // 使用公共方法检查在线状态
        {
            c_buzzer.Sound(i);// 处理第i个电机离线的情况
        }
    }
}

void RemoteState()
{
    dr16.checkRemotecontrolState();

    if(!dr16.GetIsRemotecontrolOnline())
    {
        c_buzzer.remote();
    }
}

void CreceiveState()
{
    Cboard.checkBoardCommState();

    if(!Cboard.GetIsBoardCommOnline())
    {
        c_buzzer.board();
    }
}
