#ifndef _STATE_HPP_
#define _STATE_HPP_

#include "../User/MidLayer/Managers/state_manager/state_base.hpp"

// ǰ������
namespace Clicker {
    class DR16;
}

// �ⲿ��������

extern Clicker::DR16 dr16;


namespace State
{
    class model
    {
        enum model_state
        {
            STOP,      //2,2
            FOLLOW,    //2,3
            NOTFOLLOW, //3,2
            SHOT,      //2,1
            VISION,    //1,2
            KEYBOARD,  //1,1
        };
        public:
			void updateState();  // ������ʵ����.cpp�ļ���
			model_state getCurrentState() 
			{
				return current_state;
			}

        private:
            model_state current_state = STOP;
        
    };
}

void MotorState();
void RemoteState();

#endif
