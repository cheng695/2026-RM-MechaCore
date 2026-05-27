#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "SerialTask.hpp"
#include "core/BSP/Common/StateWatch/state_watch.hpp"
#include "core/BSP/Common/StateWatch/buzzer_manager.hpp"


// 下行数据can id
constexpr uint32_t kCanIdDownlink = 0x311;

/**
 * @brief 板间 CAN 通讯类
 * @details 管理云台与底盘之间的 CAN 数据收发:
 *          - board_tx_: 云台→底盘 (遥控器 + 角度 + 目标速度)
 *          - board_rx_: 底盘→云台 (里程计 + 速度 + 热量)
 *          - 连接状态监测 (看门狗 + 离线蜂鸣)
 */
class BoardCommunication
{
    public:
        #pragma pack(1)
        // 发送：云台→底盘 下行数据帧 (CAN ID 0x311, 35 字节)
        struct BoardTx
        {
            uint8_t  rc_data[18];   // 18  遥控器数据转发 (DT7Rx_buffer)
            float    angle;         // 4   Motor6020 角度 (rad)
            bool     scroll;        // 1   发射机构 scroll 状态
            float    target_vx;     // 4   底盘目标 x 线速度 (m/s)
            float    target_vy;     // 4   底盘目标 y 线速度 (m/s)
            float    target_wz;     // 4   底盘目标角速度 (rad/s)
        };

        // 接收：底盘→云台 上行数据帧 (CAN ID 0x310, 28 字节)
        struct BoardRx
        {
            float x;           // 4  里程计 x (m)
            float y;           // 4  里程计 y (m)
            float yaw;         // 4  偏航角 (rad), CCW 为正
            float vx;          // 4  底盘系 x 线速度 (m/s)
            float vy;          // 4  底盘系 y 线速度 (m/s)
            float wz;          // 4  角速度 (rad/s)

            uint16_t shooter_barrel_cooling_value;  // 2  枪口每秒冷却值
            uint16_t shooter_barrel_heat_limit;     // 2  枪口热量上限
        };
        #pragma pack()

        BoardTx board_tx_;
        BoardRx board_rx_;

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

        // ──── 上行数据访问 (底盘→云台) ────

        float GetChassisYaw() const { return board_rx_.yaw; }
        float GetChassisVx()  const { return board_rx_.vx; }
        float GetChassisVy()  const { return board_rx_.vy; }
        float GetChassisWz()  const { return board_rx_.wz; }

        uint16_t GetHeatLimit() const { return board_rx_.shooter_barrel_heat_limit; }
        uint16_t GetHeatCool()  const { return board_rx_.shooter_barrel_cooling_value; }

    private:
        BSP::WATCH_STATE::StateWatch statewatch_;
};

/**
 * @brief 导航通讯类
 * @details 管理云台与 miniPC 之间的导航数据收发, 发送里程计+速度数据, 接收目标速度
 */
class Navigation
{
    public:
        #pragma pack(1)
        struct Frame
        {
            uint8_t head = 0;
        };

        struct Tx_Odom
        {
            float x = 0.0f;         // 4  里程计 x (m)
            float y = 0.0f;         // 4  里程计 y (m)
            float yaw = 0.0f;       // 4  偏航角 (rad)
        };

        struct Tx_Vel
        {
            float vx = 0.0f;        // 4  底盘系 x 线速度 (m/s)
            float vy = 0.0f;        // 4  底盘系 y 线速度 (m/s)
            float wz = 0.0f;        // 4  角速度 (rad/s)
        };

        struct Tx_Other
        {
            uint32_t time = 0;      // 4  时间戳 (ms)
            uint16_t checksum = 0;  // 2  前 29 字节累加和低 16 位
        };

        struct Rx_Frame
        {
            uint8_t head = 0;
        };

        struct Rx_Target
        {
            float vx = 0.0f;        // 4  目标 x 线速度 (m/s)
            float vy = 0.0f;        // 4  目标 y 线速度 (m/s)
            float wz = 0.0f;        // 4  目标角速度 (rad/s)
        };

        struct Rx_Other
        {
            uint16_t checksum = 0;  // 2  前 13 字节累加和低 16 位
        };
        #pragma pack()

        // 发送相关
        Frame frame;
        Tx_Odom tx_odom;
        Tx_Vel tx_vel;
        Tx_Other tx_other;

        // 接收相关
        Rx_Frame rx_frame;
        Rx_Target rx_target;
        Rx_Other rx_other;

        uint8_t Tx_pData[31] = {0};
        uint8_t Rx_pData[15] = {0};

    public:
        // 组装并发送一帧导航数据到 miniPC (USB CDC)
        void Data_send();

        // 解析接收到的目标速度数据帧
        void dataReceive();

        /// 获取 miniPC 下发的目标速度
        float getTargetVx() const { return rx_target.vx; }
        float getTargetVy() const { return rx_target.vy; }
        float getTargetWz() const { return rx_target.wz; }

        /// 设置发送的里程计数据
        void setOdom(float x, float y, float yaw)
        {
            tx_odom.x = x; tx_odom.y = y; tx_odom.yaw = yaw;
        }

        /// 设置发送的速度数据
        void setVel(float vx, float vy, float wz)
        {
            tx_vel.vx = vx; tx_vel.vy = vy; tx_vel.wz = wz;
        }
};

extern Navigation navigation;

// ──── 视觉 USB 虚拟串口通讯协议 ─────────────────────────────────────────
// 云台通过 USB CDC 与 miniPC 通讯, 发送 IMU 四元数 + Pitch 角度等, 接收目标角度

/**
 * @brief 视觉通讯类
 * @details 管理云台与 miniPC 之间的数据收发, 包括四元数姿态发送、目标角度接收、
 *          发射指令解析等
 */
class Vision
{
    public:
        // 发送帧头
        struct Frame
        {
            uint8_t head_one = 0;
            uint8_t head_two = 0;
        };

        // 发送: 云台姿态数据
        struct Tx_Gimbal
        {
            float quat_w = 0.0f;
            float quat_x = 0.0f;
            float quat_y = 0.0f;
            float quat_z = 0.0f;
            uint32_t time = 0;          // 时间戳 (ms)
        };

        // 发送: 其他信息
        struct Tx_Other
        {
            uint8_t bullet_rate = 0;    // 射速
            uint8_t enemy_color = 0;    // 敌方颜色
            uint8_t vision_mode = 0;    // 视觉模式
            uint8_t tail = 0;           // 帧尾
        };

        // 接收帧头
        struct Rx_Frame
        {
            uint8_t head_one = 0;
            uint8_t head_two = 0;
        };

        // 接收: 视觉解算的目标角度
        struct Rx_Target
        {
            float pitch_angle = 0.0f;   // 目标 Pitch 角度 (deg)
            float yaw_angle = 0.0f;     // 目标 Yaw 角度 (deg)
            uint32_t time = 0;          // 时间戳 (ms)
        };

        // 接收: 视觉状态与控制信息
        struct Rx_Other
        {
            uint8_t vision_ready = 0;   // 视觉就绪标志
            uint8_t fire = 0;           // 发射指令
            uint8_t tail = 0;           // 帧尾
            uint8_t aim_x = 0;          // 瞄准点 x
            uint8_t aim_y = 0;          // 瞄准点 y
        };

        // 缓存的本地目标角度
        float pitch_angle_ = 0.0f;
        float yaw_angle_ = 0.0f;

        // 发送相关
        Frame frame;
        Tx_Gimbal tx_gimbal;
        Tx_Other tx_other;

        // 接收相关
        Rx_Frame rx_frame;
        Rx_Target rx_target;
        Rx_Other rx_other;

        uint8_t Tx_pData[26] = {0};     // 发送缓冲区 (26 字节)
        uint8_t Rx_pData[19] = {0};     // 接收缓冲区 (19 字节)

        bool fire_flag = false;
        uint32_t fire_num = 0;
        bool vision_flag = false;       // 视觉在线标志
        uint8_t vision_mode = 0;        // 当前视觉模式

        // 发射指令变化检测
        bool fire_value_initialized = false;
        uint32_t fire_update_count = 0;

        // 调试用: 缓存最近一次的接收数据
        Rx_Target debug_rx_target;
        Rx_Other debug_rx_other;

    public:
        // 组装并发送一帧数据到 miniPC (USB CDC)
        void Data_send();

        // 解析接收到的视觉数据帧
        void dataReceive();

        // 设置本地 Pitch 目标角度
        float set_pitch_angle(float yaw_angle)
        {
            return yaw_angle_ = yaw_angle;
        }

        // 设置本地 Yaw 目标角度
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

        // 获取视觉解算的 Yaw 角度
        float getVisionYaw()
        {
            return rx_target.yaw_angle;
        }

        // 获取视觉解算的 Pitch 角度
        float getVisionPitch()
        {
            return rx_target.pitch_angle;
        }

        // 视觉是否就绪
        bool getVisionReady()
        {
            return rx_other.vision_ready;
        }

        // 获取发射指令
        bool get_fire_num()
        {
            return rx_other.fire;
        }

        // 设置发送给视觉的模式
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
