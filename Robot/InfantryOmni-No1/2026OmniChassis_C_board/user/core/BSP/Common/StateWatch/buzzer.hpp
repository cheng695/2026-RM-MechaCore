#ifndef BUZZER_HPP
#define BUZZER_HPP

#include "tim.h"
#include "cmsis_os.h"

namespace  BSP::WATCH_STATE 
{
    class buzzer
    {
        public:
            buzzer(){}

            void B_()
            {
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 200);
                osDelay(100);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
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
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 200);
                osDelay(500);
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
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


        private:


    };
    
}

#endif
