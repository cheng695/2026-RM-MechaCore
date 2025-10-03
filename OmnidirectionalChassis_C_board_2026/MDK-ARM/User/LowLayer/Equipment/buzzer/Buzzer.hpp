#ifndef _BUZZER_HPP_
#define _BUZZER_HPP_ 

#include "../User/LowLayer/HAL_/pwm/pwm_driver.hpp"
#include "FreeRTOS.h"
#include <cmsis_os.h>

extern PwmDriver::pwmDriver<1, 4> myPwmDriver;

namespace Buzzer
{
    class C_buzzer
    {
        public:
            void m1(); //B_1
            void m2(); //B_2
            void m3(); //B_3
            void m4(); //B_4
            void to_bord(); //__B_B_B__
            void imu(); //_B_B__

            typedef void(C_buzzer::*BuzzerSound)();

            static const BuzzerSound buzzer_sound[4];

            void Sound(int index);
    };
}
#endif
