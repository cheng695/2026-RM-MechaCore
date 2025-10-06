#ifndef _STATE_BASE_HPP_
#define _STATE_BASE_HPP_

#include "main.h"

namespace State
{
    class monitoring
    {
        public:
            int lastUpdateTime;
            int currentTime;
            bool isOnline;
            
            void getlastTime()
            {
                currentTime = HAL_GetTick();
            }

            void getTime()
            {
                lastUpdateTime = HAL_GetTick();
            }
            
            bool checkTime(int time)
            {
                if (currentTime - lastUpdateTime > time)
                {
                    isOnline = false;
                }
                return isOnline;
            }
    };
}

#endif
