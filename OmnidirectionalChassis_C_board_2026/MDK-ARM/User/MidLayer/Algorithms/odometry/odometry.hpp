#include <vector>
#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_ 

namespace odometry
{
    class angle_odometry
    {
        public:
            angle_odometry(float MaxAngle, float Resolution) : maxAngle(MaxAngle), Resolution(Resolution)
            {
                accumulatedAngle = 0.0f;
                lastAngleRaw = 0.0f;
                Resolution_coefficient = MaxAngle / Resolution;
                initialized = false;
            }

            float update(float currentRawAngle)
            {
                currentRawAngle *= Resolution_coefficient;

                if (!initialized) 
                {
                    lastAngleRaw = currentRawAngle;
                    accumulatedAngle = currentRawAngle;
                    initialized = true;
                    return accumulatedAngle;
                }
                
                // 处理角度翻转情况
                float delta = currentRawAngle - lastAngleRaw;
                const float halfMax = maxAngle / 2.0f;
                
                if (delta > halfMax) 
                {
                    // 向下翻转
                    accumulatedAngle += (-maxAngle + delta);
                } 
                else if (delta < -halfMax) 
                {
                    // 向上翻转
                    accumulatedAngle += (maxAngle + delta);
                }
                else 
                {
                    // 正常变化
                    accumulatedAngle += delta;
                }
                
                lastAngleRaw = currentRawAngle;
                return accumulatedAngle;
            }

        private:
            float maxAngle;           // 最大角度值
            float Resolution;         // 编码器分辨率
            float accumulatedAngle;   // 累计角度
            float lastAngleRaw;       // 上一次原始角度值
            float Resolution_coefficient; //分辨率系数
            bool initialized;         // 初始化标志
    };
}

#endif
