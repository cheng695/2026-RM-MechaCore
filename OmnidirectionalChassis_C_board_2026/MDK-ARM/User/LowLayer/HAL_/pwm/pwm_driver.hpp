#ifndef _PWM_DRIVER_HPP_
#define _PWM_DRIVER_HPP_ 

#include "tim.h"

namespace PwmDriver
{
    template<uint8_t N, uint8_t M> class pwmDriver
    {
        public:
            pwmDriver(TIM_HandleTypeDef* (&htim)[N], uint32_t (&channel)[M]) 
                : htim_(htim), channel_(channel){}

            void setPwm(uint8_t myhtim, uint8_t mychannel, uint16_t duty)
            {
                if (myhtim < N) 
                {
                    __HAL_TIM_SET_COMPARE(htim_[myhtim], channel_[mychannel], duty);
                }
            }

            void PWMInit()
            {
                for(uint8_t i = 0; i < N; i++)
                {
                    for(uint8_t j = 0; j < M; j++)
                    {
                        HAL_TIM_PWM_Start_IT(htim_[i], channel_[j]);
                    }
                }
            }

        private:
            TIM_HandleTypeDef* (&htim_)[N];
            uint32_t (&channel_)[M];
    };
}

#ifdef __cplusplus
extern "C" {
#endif

void pwminit();

#ifdef __cplusplus
}
#endif

#endif
