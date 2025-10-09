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
        if(!Motor3508.GetIsMotorOnline(i))  // ʹ�ù��������������״̬
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
    // ���ö���ļ�鷽���������е��״̬
    Motor3508.checkMotorsState();
    
    // Ȼ����ÿ�����������״̬
    for(int i = 0; i < Motor3508.GetMotorCount(); i++)
    {
        if(!Motor3508.GetIsMotorOnline(i))  // ʹ�ù��������������״̬
        {
            c_buzzer.Sound(i);// �����i��������ߵ����
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
