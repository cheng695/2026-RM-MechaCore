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
    class ChassisType
    {
        enum chassis_type
        {
            Mecanum,            //�����ķ��
            Omnidirectional,    //ȫ����
            Steering,           //����
            WheelLeg            //����
        };
    };

    class model
    {
        public:
            enum model_state
            {
                STOP,              //2,2(ȫͣ)
                FOLLOW,            //2,3(���̸��棬���������С����)
                NOTFOLLOW,         //3,2(���̲����棬���������С����)
                SHOT_FOLLOW,       //2,1(���̸������)
                SHOT_Rotate,       //3,1(С�������)
                VISION_NOTSHOT,    //1,2(���̲����棬�Ӿ������������С����)
                VISION_SHOT,       //1,3(���̲����棬�Ӿ������)
                KEYBOARD,          //1,1(����ģʽ)
            };

			void updateState();  // ������ʵ����.cpp�ļ���
			model_state getCurrentState() 
			{
				return current_state;
			}

            model_state getLastState() 
			{
				return last_state;
			}

            bool isStateChanged() 
			{
				return is_statechanged;
			}

            void SetcurrentState(model_state state)
            {
                current_state = state;
            }
        private:
            model_state last_state = STOP;   //��һ��״̬
            model_state current_state = STOP; //��ǰ״̬
            bool is_statechanged = false;
    };
}

void MotorState();
void RemoteState();

#endif
