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
            
            float getTime()
            {
                currentTime = HAL_GetTick();
                return currentTime;
            }

            float getlastTime()
            {
                lastUpdateTime = HAL_GetTick();
                return lastUpdateTime;
            }
            
            bool checkTime(int time)
            {
                if (currentTime - lastUpdateTime > time)
                {
                    isOnline = false;
                }
                else 
                {
                    isOnline = true;  // ��ʱ�䷶Χ�ڣ�����Ϊ����
                }
                return isOnline;
            }
    };
}

#endif
