#ifndef _REMOTE_TARGET_HPP_
#define _REMOTE_TARGET_HPP_

#include "../User/LowLayer/Equipment/remote/Dr16.hpp"

extern Clicker::DR16 dr16;

namespace RemoteTarget
{
    class remotetarget
    {
        public:
            bool dead_zone_flag0;
            bool dead_zone_flag1;
            bool dead_zone_flag2;
            bool dead_zone_flag3;
            bool dead_zone_flag4;
            
            void TargetDataUpdate();

            float Getvx_left()
            {
                return vx_left;
            }

            float Getvy_left()
            {
                return vy_left;
            }

            float Getvx_right()
            {
                return vx_right;
            }

            float Getvy_right()
            {
                return vy_right;
            }

            float GetRoller()
            {
                return Roller;
            }
            
        private:
            float vx_left;
            float vy_left;
            float vx_right; 
            float vy_right;
            float Roller;

            float w;
            float a;
            float s;
            float d;
            float q;
            float e;

    };
}

void Dead_zone(); 

#endif
