#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/Task/SerialTask.hpp"
#include "../User/core/BSP/Common/StateWatch/state_watch.hpp"
#include "../User/core/BSP/Common/StateWatch/buzzer_manager.hpp"

extern uint8_t BoardRx[4];;

class BoardCommunication
{
    public:
        BoardCommunication(int timeThreshold = 100) : statewatch_(timeThreshold) 
        {
        }
        virtual ~BoardCommunication() = default;

        void updateTimestamp()
        {
            statewatch_.UpdateLastTime();
        }

        bool isConnected()
        {
            statewatch_.UpdateTime();
            statewatch_.CheckStatus();
            if(statewatch_.GetStatus() == BSP::WATCH_STATE::Status::OFFLINE)
            {
                BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing();
            }
            return statewatch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
        }

    private:
        BSP::WATCH_STATE::StateWatch statewatch_;
};



class Vision
{
    public:
        /* data */
        struct Frame
        {
            uint8_t head_one;
            uint8_t head_two;
        };

        struct Tx_Gimbal
        {
            int32_t pitch_angle;
            int32_t yaw_angle;
            uint32_t time;
        };

        struct Tx_Other
        {
            uint8_t bullet_rate;
            uint8_t enemy_color;
            uint8_t vision_mode;
            uint8_t tail;
        };

        struct Rx_Frame
        {
            uint8_t head_one;
            uint8_t head_two;
        };

        struct Rx_Target
        {
            float pitch_angle;
            float yaw_angle;

            float time;
        };

        struct Rx_Other
        {
            uint8_t vision_ready;
            uint8_t fire;
            uint8_t tail;

            uint8_t aim_x;
            uint8_t aim_y;
        };

        float pitch_angle_; //度
        float yaw_angle_;   //度

        Frame frame;
        Tx_Gimbal tx_gimbal;
        Tx_Other tx_other;

        Rx_Frame rx_frame;
        Rx_Target rx_target;
        Rx_Other rx_other;

        uint8_t Tx_pData[18];
        uint8_t Rx_pData[19];

        bool fire_flag;
        uint32_t fire_num;

        bool vision_flag; // 超过一定范围就置1
        uint8_t vision_mode;  // 0：停火，1：单发，2：连发

    public:
        void Data_send();
        void dataReceive();
        void time_demo();

        float set_pitch_angle(float yaw_angle)
        {
            return yaw_angle_ = yaw_angle;
        }

        float set_yaw_angle(float pitch_angle)
        {
            return pitch_angle_ = pitch_angle;
        }

        float getTarYaw()
        {
            return yaw_angle_;
        }

        float getTarPitch()
        {
            return pitch_angle_;
        }

        float getVisionYaw()
        {
            return rx_target.yaw_angle;
        }

        float getVisionPitch()
        {
            return rx_target.pitch_angle;
        }

        bool getVisionReady()
        {
            return rx_other.vision_ready;
        }

        bool get_fire_num()
        {
            return rx_other.fire;
        }

        void setVisionMode(uint8_t mode)
        {
            tx_other.vision_mode = mode;
        }

        uint8_t getAimX()
        {
            return rx_other.aim_x;
        }

        uint8_t getAimY()
        {
            return rx_other.aim_y;
        }

        bool getVisionFlag()
        {
            return vision_flag;
        }

        uint8_t getVisionMode()
        {
            return vision_mode;
        }
};

#endif
