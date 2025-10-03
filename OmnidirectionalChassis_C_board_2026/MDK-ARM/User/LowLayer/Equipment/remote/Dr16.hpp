#ifndef _DR16_HPP_
#define _DR16_HPP_

#include "usart.h"
#include "dma.h"
#include "string.h"
#include "../User/MidLayer/Managers/state_manager/state_base.hpp"
#include "../User/LowLayer/HAL_\uart\uart_driver.hpp"

namespace Clicker
{

    class DR16 : public State::monitoring
    {
        public:

            struct 
            { 
                uint16_t ch1; //ÓÒ±ßx
                uint16_t ch0; //ÓÒ±ßy
                uint16_t ch3; //×ó±ßx  ¶«±±Ìì
                uint16_t ch2; //×ó±ßy
                uint8_t s1;
                uint8_t s2;
            }rc;
            struct 
            {
                int16_t x;
                int16_t y;
                int16_t z;
                uint8_t press_l;
                uint8_t press_r;
            }mouse;
            struct 
            {
                char w;
                char s;
                char a;
                char d;
                char q;
                char e;
                char shift;
                char ctrl;
            }key;
            struct
            {
                uint16_t reserved;
            }reserved_;

        void DR16DataUpdata(const uint8_t *pData);
    };
}

#endif
