#include "state.hpp"
#include "../User/LowLayer/Equipment/motor/djimotor.hpp"
#include "../User/LowLayer/Equipment/remote/dr16.hpp"

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
}


void MotorState()
{
    M3508_201.getlastTime();
    M3508_201.checkTime(200);
    if(!M3508_201.isOnline)
    {

    }

    M3508_202.getlastTime();
    M3508_202.checkTime(200);
    if(!M3508_202.isOnline)
    {
        
    }

    M3508_203.getlastTime();
    M3508_203.checkTime(200);
    if(!M3508_203.isOnline)
    {
        
    }

    M3508_204.getlastTime();
    M3508_204.checkTime(200);
    if(!M3508_204.isOnline)
    {
        
    }
}

void RemoteState()
{
    dr16.getlastTime();
    dr16.checkTime(200);
    if(!dr16.isOnline)
    {

    }
}
