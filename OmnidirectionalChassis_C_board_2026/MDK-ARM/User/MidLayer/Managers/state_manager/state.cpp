#include "state.hpp"
#include "../User/LowLayer/Equipment/motor/djimotor.hpp"
#include "../User/LowLayer/Equipment/remote/dr16.hpp"

extern Buzzer::C_buzzer c_buzzer;

State::model chass_model;

void State::model::updateState() 
{
    // ������԰�ȫ��ʹ�� dr16����Ϊ��.cpp�ļ��а�������������
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

    // ���ö���ļ�鷽���������е��״̬
    Motor3508.checkMotorsState();
    
    // Ȼ����ÿ�����������״̬
    for(int i = 0; i < Motor3508.GetMotorCount(); i++)
    {
        if(!Motor3508.IsMotorOnline(i))  // ʹ�ù��������������״̬
        {
            c_buzzer.Sound(i);// �����i��������ߵ����
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
