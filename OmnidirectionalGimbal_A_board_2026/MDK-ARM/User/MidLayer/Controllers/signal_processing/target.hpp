#ifndef _TARGET_HPP_
#define _TARGET_HPP_

#include "../User/LowLayer/Equipment/remote/Dr16.hpp"

extern Clicker::DR16 dr16;

namespace Target
{
    class target
    {
        public:
            bool dead_zone_flag0;
            bool dead_zone_flag1;
            bool dead_zone_flag2;
            bool dead_zone_flag3;
            bool dead_zone_flag4;
            
            void TargetDataUpdate();
            
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

#endif
