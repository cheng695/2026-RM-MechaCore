#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "SerialTask.hpp"
#include "core/BSP/Common/StateWatch/state_watch.hpp"
#include "core/BSP/Common/StateWatch/buzzer_manager.hpp"

extern uint8_t BoardRx[4];

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

        void SetHeatLimit(uint8_t *data)
        {
            memcpy(&shooter_barrel_heat_limit, data, sizeof(uint16_t));
        }

        void SetHeatCool(uint8_t *data)
        {
            memcpy(&shooter_barrel_cooling_value, data, sizeof(uint16_t));
        }

        uint16_t GetHeatLimit()
        {
            return shooter_barrel_heat_limit;
        }

        uint16_t GetHeatCool()
        {
            return shooter_barrel_cooling_value;
        }

    private:
        BSP::WATCH_STATE::StateWatch statewatch_;
        uint16_t shooter_barrel_heat_limit;
        uint16_t shooter_barrel_cooling_value;
};

class Vision
{
    public:
        struct Frame
        {
            uint8_t head_one = 0;
            uint8_t head_two = 0;
        };

        struct Tx_Gimbal
        {
            float quat_w = 0.0f;
            float quat_x = 0.0f;
            float quat_y = 0.0f;
            float quat_z = 0.0f;
            uint32_t time = 0;
        };

        struct Tx_Other
        {
            uint8_t bullet_rate = 0;
            uint8_t enemy_color = 0;
            uint8_t vision_mode = 0;
            uint8_t tail = 0;
        };

        struct Rx_Frame
        {
            uint8_t head_one = 0;
            uint8_t head_two = 0;
        };

        struct Rx_Target
        {
            float pitch_angle = 0.0f;
            float yaw_angle = 0.0f;
            uint32_t time = 0;
        };

        struct Rx_Other
        {
            uint8_t vision_ready = 0;
            uint8_t fire = 0;
            uint8_t tail = 0;
            uint8_t aim_x = 0;
            uint8_t aim_y = 0;
        };

        float pitch_angle_ = 0.0f;
        float yaw_angle_ = 0.0f;

        Frame frame;
        Tx_Gimbal tx_gimbal;
        Tx_Other tx_other;

        Rx_Frame rx_frame;
        Rx_Target rx_target;
        Rx_Other rx_other;

        uint8_t Tx_pData[26] = {0};
        uint8_t Rx_pData[19] = {0};

        bool fire_flag = false;
        uint32_t fire_num = 0;
        bool vision_flag = false;
        uint8_t vision_mode = 0;

        bool fire_value_initialized = false;
        uint32_t fire_update_count = 0;
        Rx_Target debug_rx_target;
        Rx_Other debug_rx_other;

    public:
        void Data_send();
        void dataReceive();

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
