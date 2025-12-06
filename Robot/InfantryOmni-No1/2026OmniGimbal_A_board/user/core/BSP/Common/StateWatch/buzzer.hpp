// buzzer.hpp - 添加静态方法
#ifndef BUZZER_HPP
#define BUZZER_HPP

#include "tim.h"
#include "cmsis_os.h"

namespace BSP::WATCH_STATE 
{
    class buzzer
    {
    public:
        buzzer(){}
        
        // 保持原有方法不变
        void B_()
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 296);
            osDelay(100);
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
            osDelay(100);
        }
        
        void B_B_()
        {
            for(int i = 0; i < 2; i++)
            {
                B_();
            }
        }
        
        void B_B_B_()
        {
            for(int i = 0; i < 3; i++)
            {
                B_();
            }
        }
        
        void B_B_B_B_()
        {
            for(int i = 0; i < 4; i++)
            {
                B_();
            }
        }
        
        void B___()
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 296);
            osDelay(500);
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
            osDelay(500);
        }
        
        void ring(int i)
        {
            switch (i)
            {
                case 1:
                    B_();
                    osDelay(200);
                    break;
                case 2:
                    B_B_();
                    osDelay(200);
                    break;
                case 3:
                    B_B_B_();
                    osDelay(200);
                    break;
                case 4:
                    B_B_B_B_();
                    osDelay(200);
                    break;
                default:
                    break;
            }
        }
        
        // 添加静态方法，方便全局访问
        static void ringStatic(uint8_t id)
        {
            static buzzer instance;
            if (id >= 1 && id <= 4)
            {
                instance.ring(id);
            }
            else if (id == 0xFF) // 遥控器
            {
                instance.B___();
            }
        }
        
    private:
        
    };
}
#endif
