#include "pwm_driver.hpp"

TIM_HandleTypeDef* pwmTimers[1] = {&htim4};
uint32_t pwmChannels[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

PwmDriver::pwmDriver<1, 4> myPwmDriver(pwmTimers, pwmChannels);

extern "C" void pwminit()
{
    myPwmDriver.PWMInit();
}
