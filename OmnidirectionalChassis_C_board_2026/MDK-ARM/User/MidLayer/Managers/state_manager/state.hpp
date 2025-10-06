#ifndef _STATE_HPP_
#define _STATE_HPP_

#include "../User/MidLayer/Managers/state_manager/state_base.hpp"

// 前向声明
namespace Clicker {
    class DR16;
}

// 外部变量声明

extern Clicker::DR16 dr16;


namespace State
{
    class ChassisType
    {
        enum chassis_type
        {
            Mecanum,            //麦克纳姆轮
            Omnidirectional,    //全向轮
            Steering,           //舵轮
            WheelLeg            //轮腿
        };
    };

    class model
    {
        public:
            enum model_state
            {
                STOP,              //2,2(全停)
                FOLLOW,            //2,3(底盘跟随，不射击，可小陀螺)
                NOTFOLLOW,         //3,2(底盘不跟随，不射击，可小陀螺)
                SHOT_FOLLOW,       //2,1(底盘跟随射击)
                SHOT_Rotate,       //3,1(小陀螺射击)
                VISION_NOTSHOT,    //1,2(底盘不跟随，视觉，不射击，可小陀螺)
                VISION_SHOT,       //1,3(底盘不跟随，视觉，射击)
                KEYBOARD,          //1,1(键鼠模式)
            };

			void updateState();  // 声明，实现在.cpp文件中
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
            model_state last_state = STOP;   //上一个状态
            model_state current_state = STOP; //当前状态
            bool is_statechanged = false;
    };
}

void MotorState();
void RemoteState();

#endif
