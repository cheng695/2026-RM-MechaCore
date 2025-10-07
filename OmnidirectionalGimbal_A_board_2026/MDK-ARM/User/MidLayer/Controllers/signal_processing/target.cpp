#include "target.hpp"

Target::target dr16_tar;

void Target::target::TargetDataUpdate()
{
    
    dr16_tar.vx_left   = dead_zone_flag3 ? 0.0f : (dr16.rc.ch3 - 1024.0f)/660.0f;
    dr16_tar.vy_left   = dead_zone_flag2 ? 0.0f : (dr16.rc.ch2 - 1024.0f)/660.0f;
    dr16_tar.vx_right  = dead_zone_flag1 ? 0.0f : (dr16.rc.ch1 - 1024.0f)/660.0f;
    dr16_tar.vy_right  = dead_zone_flag0 ? 0.0f : (dr16.rc.ch0 - 1024.0f)/660.0f;
    dr16_tar.Roller    = (dr16.reserved_.reserved - 1024.0f)/660.0f;
}


float channels[] = {(float)dr16.rc.ch0, (float)dr16.rc.ch1, (float)dr16.rc.ch2, (float)dr16.rc.ch3, (float)dr16.reserved_.reserved};
bool* flags[] = {&dr16_tar.dead_zone_flag0, &dr16_tar.dead_zone_flag1, &dr16_tar.dead_zone_flag2, &dr16_tar.dead_zone_flag3, &dr16_tar.dead_zone_flag4};

void Dead_zone() 
{
    for(int i = 0; i < 5; i++) 
    {
        float diff = channels[i] - 1024.0f;
        *(flags[i]) = (diff < 15.0f && diff > -15.0f);
    }
}
