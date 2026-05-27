#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "SerialTask.hpp"
#include "core/BSP/Common/StateWatch/state_watch.hpp"
#include "core/BSP/Common/StateWatch/buzzer_manager.hpp"

extern uint8_t BoardRx[4];

// ──── 板间 CAN 通讯协议 (底盘→云台) ─────────────────────────────────────
// 底盘通过 CAN2 发送 UplinkPacket_TX (34 字节), CAN ID 0x310, CANTransport 分包
// 云台解析导航数据 + 裁判系统热量数据

#pragma pack(1)
struct NavigationData_RX
{
    float x;           // 4  里程计 x (m)
    float y;           // 4  里程计 y (m)
    float yaw;         // 4  偏航角 (rad), CCW 为正
    float vx;          // 4  底盘系 x 线速度 (m/s)
    float vy;          // 4  底盘系 y 线速度 (m/s)
    float wz;          // 4  角速度 (rad/s)
    uint32_t stamp_ms; // 4  MCU 时间戳 (ms)
    uint16_t checksum; // 2  前 28 字节累加和低 16 位
};
static_assert(sizeof(NavigationData_RX) == 30, "NavigationData_RX size mismatch");

struct RefereeData_RX
{
    uint16_t shooter_barrel_cooling_value; // 2  枪口每秒冷却值
    uint16_t shooter_barrel_heat_limit;    // 2  枪口热量上限
};
static_assert(sizeof(RefereeData_RX) == 4, "RefereeData_RX size mismatch");

/// 底盘→云台 上行合包 (CAN ID 0x310, CANTransport 分包)
struct UplinkPacket_RX
{
    NavigationData_RX nav;
    RefereeData_RX   referee;
};
static_assert(sizeof(UplinkPacket_RX) == 34, "UplinkPacket_RX size mismatch");
#pragma pack()

extern UplinkPacket_RX gimbal_uplink;

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

        void SetHeatData(const RefereeData_RX *data)
        {
            shooter_barrel_heat_limit = data->shooter_barrel_heat_limit;
            shooter_barrel_cooling_value = data->shooter_barrel_cooling_value;
        }

        void SetNavData(const NavigationData_RX *data)
        {
            chassis_nav_ = *data;
        }

        const NavigationData_RX &GetNavData() const { return chassis_nav_; }
        float GetChassisYaw() const { return chassis_nav_.yaw; }
        float GetChassisVx() const  { return chassis_nav_.vx; }
        float GetChassisVy() const  { return chassis_nav_.vy; }
        float GetChassisWz() const  { return chassis_nav_.wz; }

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
        NavigationData_RX chassis_nav_{};
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
