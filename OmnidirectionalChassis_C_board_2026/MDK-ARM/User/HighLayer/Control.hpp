#ifndef _CONTROL_HPP_
#define _CONTROL_HPP_ 

#include "../User/MidLayer/Controllers/chassis_control/chassis_control.hpp"

extern ChassisControl::modelchoose model_choose;
extern ChassisControl::Send motor_send;
extern ChassisControl::PIDCalculate pid_calculate;
extern PidTarget::pidtarget chassis_pidtarget;

namespace Control
{
    class motor_control
    {
        public:
            void wheelControl();
    };

    class remote_control
    {
        public:
            void remoteControl();
    };
}


#endif
