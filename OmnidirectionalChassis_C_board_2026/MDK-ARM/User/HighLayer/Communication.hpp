#ifndef _COMMUNICATION_HPP_
#define _COMMUNICATION_HPP_ 

#include "../User/MidLayer/Managers/comm_manager/Callback.hpp"
#include "../User/MidLayer/Managers/comm_manager/CtoA.hpp"

extern BoardComm::CtoA Cboard;

namespace Communication
{
    class communication
    {
        public:
            void sendBoard();
            void sendTools();
    };
}

#endif
