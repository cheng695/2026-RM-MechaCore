#ifndef _MONITOR_HPP_
#define _MONITOR_HPP_ 

#include "../User/MidLayer/Managers/state_manager/state.hpp"

namespace Monitor
{
    class motor_monitor
    {
        public:
            void MotorMonitor();
    };

    class remote_monitor
    {
        public:
            void RemotecontrolMonitor();
    };

    class board_monitor
    {
        public:
            void BoardMonitor();
    };
}

#endif
