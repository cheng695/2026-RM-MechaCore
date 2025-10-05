#ifndef _CHASSIS_CONTROL_HPP_
#define _CHASSIS_CONTROL_HPP_ 

#include "../User/MidLayer/Managers/state_manager/state.hpp"
#include "../User/MidLayer/Controllers/signal_processing/target.hpp"

extern State::model chass_model;
extern Target::target dr16_tar;

namespace ChassisControl
{
    class modelchoose
    {
        public:
            void ModeSelection();
            
        private:
            
    };
}

void nofollow(float phi);

#endif
