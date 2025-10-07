#ifndef _MOTOR_BASE_HPP_
#define _MOTOR_BASE_HPP_ 

#include <array>
#include <memory>
#include <unordered_map>
#include "../User/LowLayer/HAL_/can/can_driver.hpp"
#include "../User/MidLayer/Managers/state_manager/state_base.hpp"
#include "../User/MidLayer/Algorithms/odometry/odometry.hpp"

extern odometry::angle_odometry* Accumulate_angle;

namespace motor
{
    template <uint8_t N> class MotorBase : public State::monitoring

    {
        protected:
            struct EncoderData
            {
                int16_t ref_angle;              //原始角度
                int16_t ref_speed;              //原始速度
                int16_t ref_torqueCurrent;      //原始电流
                int16_t ref_torque;             //原始扭矩
                uint8_t ref_temperate;          //原始温度

                float angle_deg;                //转子角度 角度
                float angle_rad;                //转子角度 弧度            
                float speed_rpm;                //转子速度 rpm
                float speed_rad_s;              //转子速度 弧度/s
                float torquecurrent;            //实际扭矩电流 A
                float torque;                   //电机扭矩 nm

                odometry::angle_odometry* Accumulate_angle_;
                float add_angle;

                bool isOnline;
                float target;
                uint32_t lasttime;

                // 构造函数初始化所有成员
                EncoderData(odometry::angle_odometry* angle = nullptr) : 
                    ref_angle(0), ref_speed(0), ref_torqueCurrent(0), ref_torque(0), ref_temperate(0),
                    angle_deg(0.0f), angle_rad(0.0f), speed_rpm(0.0f), speed_rad_s(0.0f),
                    torquecurrent(0.0f), torque(0.0f),Accumulate_angle_(Accumulate_angle),
                    add_angle(0.0f), isOnline(false), target(0.0f), lasttime(0) {}
            };
            EncoderData encoderdata[N];
        
        public:

            // 添加公共访问方法
            const EncoderData& GetEncoderData(uint8_t index) const 
            {
                if (index < N) {
                    return encoderdata[index];
                }
                return encoderdata[0];
            }
            
            bool GetIsMotorOnline(uint8_t index) const
            {
                if (index < N) 
                {
                    return encoderdata[index].isOnline;
                }
                return false;
            }

            uint8_t GetMotorCount() const
            {
                return N;
            }

            void SetTarget(uint8_t id, float target)
            {
                encoderdata[id].target = target;
            }

            float GetSpeedRef(uint8_t id)
            {
                return encoderdata[id].ref_speed;
            }

            float GetAngleRef(uint8_t id)
            {
                return encoderdata[id].ref_angle;
            }

            float GetTorqueCurrentRef(uint8_t id)
            {
                return encoderdata[id].ref_torqueCurrent;
            }

            float GetTorqueRef(uint8_t id)
            {
                return encoderdata[id].ref_torque;
            }

            float GetTemperatureRef(uint8_t id)
            {
                return encoderdata[id].ref_temperate;
            }

            float GetAngleDeg(uint8_t id)
            {
                return encoderdata[id].angle_deg;
            }

            float GetAngleRad(uint8_t id)
            {
                return encoderdata[id].angle_rad;
            }

            float GetSpeedRpm(uint8_t id)
            {
                return encoderdata[id].speed_rpm;
            }

            float GetSpeedRadS(uint8_t id)
            {
                return encoderdata[id].speed_rad_s;
            }

            float GetTorquecurrent_A(uint8_t id)
            {
                return encoderdata[id].torquecurrent;
            }

            float GetTorqueNm(uint8_t id)
            {
                return encoderdata[id].torque;
            }

    };
}


#endif
