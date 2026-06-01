#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"
#include "../User/core/BSP/Common/StateWatch/state_watch.hpp"
#include "../User/core/BSP/Common/StateWatch/buzzer_manager.hpp"
#include "../User/core/HAL/CAN/can_hal.hpp"
#include <cstring>
#include "../User/core/APP/Referee/RM_RefereeSystem.h"

// ──── 板间 CAN 通讯协议 ──────────────────────────────────────────────────
/**
 * @brief 板间通讯类
 * @details 负责处理与上位机板子之间的串口通讯，包括连接状态监测、数据解析等功能
 */
class BoardCommunication
{
    public:
        #pragma pack(1)
        // 发送：底盘→云台 上行数据帧 (CAN ID 0x310, 28 字节)
        struct BoardTX
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
        
        // 接收：上位机→底盘 下行数据帧 (CAN ID 0x311, 35 字节)
        struct BoardRX
        {
            uint8_t  rc_data[18];   // 18  遥控器数据转发 (DT7Rx_buffer)
            float    angle;         // 4   Motor6020 角度 (rad)
            bool     scroll;        // 1   发射机构 scroll 状态
            float    target_vx;     // 4   底盘目标 x 线速度 (m/s)
            float    target_vy;     // 4   底盘目标 y 线速度 (m/s)
            float    target_wz;     // 4   底盘目标角速度 (rad/s)
        };
        #pragma pack()

        BoardTX board_tx_;
        BoardRX board_rx_;

    public:
        /**
         * @brief 构造函数
         * @param timeThreshold 连接超时阈值(毫秒)，默认100ms
         */
        BoardCommunication(int timeThreshold = 100) : statewatch_(timeThreshold) 
        {
        }

        /**
         * @brief 虚析构函数
         */
        virtual ~BoardCommunication() = default;

        /**
         * @brief 更新时间戳
         * @details 记录最后一次通讯时间，用于判断连接状态
         */
        void updateTimestamp()
        {
            statewatch_.UpdateLastTime();
        }

        /**
         * @brief 检查连接状态
         * @return true-在线 false-离线
         * @details 检测通讯是否正常，若离线则触发蜂鸣器报警
         */
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

        float GetYawAngle() { return board_rx_.angle; } // 云台角度(弧度) 原始值

        /**
         * @brief 获取滤波后的云台角度
         * @details CAN 通讯频率低于控制频率，角度呈阶梯状更新。
         *          此函数检测新数据时估算角速度，无新数据时用角速度外推，使角度平滑变化。
         * @return 滤波后的云台角度(弧度)
         */
        float GetFilteredYawAngle()
        {
            float raw = board_rx_.angle;

            if (!yaw_filter_init_)
            {
                yaw_filtered_ = raw;
                yaw_last_raw_ = raw;
                yaw_cycles_ = 0;
                yaw_filter_init_ = true;
                return raw;
            }

            if (raw != yaw_last_raw_)
            {
                // 新数据到达：计算瞬时角速度并低通滤波
                float delta = raw - yaw_last_raw_;
                if (delta > 3.14159265f)       delta -= 6.28318531f;
                else if (delta < -3.14159265f) delta += 6.28318531f;

                if (yaw_cycles_ > 0)
                {
                    float instant_rate = delta / (float)yaw_cycles_;  // rad/周期
                    yaw_rate_ = yaw_rate_ * 0.8f + instant_rate * 0.2f;
                }

                yaw_last_raw_ = raw;
                yaw_cycles_ = 0;
                yaw_filtered_ = raw;
            }
            else
            {
                // 无新数据：用估算角速度外推
                yaw_cycles_++;
                yaw_filtered_ = yaw_last_raw_ + yaw_rate_ * (float)yaw_cycles_;
            }

            return yaw_filtered_;
        }

        bool GetScroll() { return board_rx_.scroll; } // 能否小陀螺 true: 不能小陀螺 false: 能小陀螺（遥控器模式下）
        float GetTargetVx() { return board_rx_.target_vx; } // 目标 x 线速度 (m/s)
        float GetTargetVy() { return board_rx_.target_vy; } // 目标 y 线速度 (m/s)
        float GetTargetWz() { return board_rx_.target_wz; } // 目标角速度 (rad/s)

    private:
        BSP::WATCH_STATE::StateWatch statewatch_;  // 连接状态监视器

        // 角度滤波器状态
        bool   yaw_filter_init_ = false;
        float  yaw_filtered_ = 0.0f;
        float  yaw_last_raw_ = 0.0f;
        float  yaw_rate_ = 0.0f;       // 角速度 (rad/周期)
        uint16_t yaw_cycles_ = 0;      // 自上次更新以来的周期数
};

/**
 * @brief 超级电容控制类
 * @details 负责超级电容的CAN通讯控制，包括状态读取和指令下发
 */
class Supercapacitor
{
    public:
        /**
         * @brief 构造函数
         * @param id_ 超级电容CAN ID
         */
        Supercapacitor(uint32_t id_)
        {
            id = id_;
            last_update_time = 0;
        } 

        /**
         * @brief 检查超级电容是否在线
         * @param timeout_ms 超时时间(ms)
         * @return true 在线
         */
        bool isConnected(uint32_t timeout_ms = 100)
        {
            return (HAL_GetTick() - last_update_time) < timeout_ms;
        } 

        /**
         * @brief 解析CAN接收数据
         * @param frame CAN数据帧
         * @details 解析来自超级电容的状态反馈数据
         */
        void Parse(const HAL::CAN::Frame &frame)
        {
            if (frame.id == 0x777)
            {
                const uint8_t* pData = frame.data;
                    
                Power_10times = (float)((int16_t)((pData[0] << 8) | pData[1]));  // 电管功率(10倍)
                CurrentEnergy = (float)((int16_t)((pData[2] << 8) | pData[3]));  // 剩余能量
                Power         = (float)((int16_t)((pData[4] << 8) | pData[5]));  // 电机+放电（正的）功率
                state         = pData[6];                                        // 工作状态
                cmd           = pData[7];                                        // 当前指令

                Power_10times /= 10;    // 转换为实际电管功率值       
                Power /= -10;           // 转换为实际电机+放电（正的）功率 
                last_update_time = HAL_GetTick();
            }
        }

        /**
         * @brief 发送数据给超级电容
         * @param RatedPower 等级功率
         * @param Instruction 超电指令 0:开启 1:关闭
         * @param BufferEnergy 缓冲能量
         */
        void sendCAN()
        {
            uint8_t send_data[8] = {0};
            int16_t power_int = (int16_t)(RatedPower);
            send_data[0] = (power_int >> 8) & 0xFF;  // 等级功率高字节
            send_data[1] = power_int & 0xFF;         // 等级功率低字节

            send_data[2] = Instruction;              // 控制指令

            int16_t buffer_int = (int16_t)BufferEnergy;
            send_data[3] = (buffer_int >> 8) & 0xFF; // 缓冲能量高字节
            send_data[4] = buffer_int & 0xFF;        // 缓冲能量低字节

            send_data[5] = isSupercapOnline;  // 超电连接标志
            send_data[6] = isRefereeOnline;   // 裁判系统连接标志
            send_data[7] = 0;  // 保留字节
            
            HAL::CAN::Frame frame;
            frame.id = id;                    // CAN ID
            frame.dlc = 8;                    // 数据长度
            memcpy(frame.data, send_data, sizeof(send_data));  // 拷贝数据
            frame.is_extended_id = false;     // 标准帧格式
            frame.is_remote_frame = false;    // 数据帧
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);  // 发送CAN帧
        }

        /**
         * @brief 设置等级功率
         * @param power 等级功率值(W)
         */
        void setRatedPower(float power)
        {
            RatedPower = power;
        }

        /**
         * @brief 设置超电指令
         * @param instruction 超电指令 0:开启 1:关闭
         */
        void setInstruction(uint8_t instruction)
        {
            Instruction = instruction;
        }

        /**
         * @brief 设置缓冲能量
         * @param energy 缓冲能量值(J)
         */
        void setBufferEnergy(float energy)
        {
            BufferEnergy = energy;
        }

        void setSupercapOnline(bool online)
        {
            isSupercapOnline = online;
        }

        void setRefereeOnline(bool online)
        {
            isRefereeOnline = online;
        }

        /**
         * @brief 获取电管功率
         * @return 功率值(W)
         */
        float GetPower_10times() { return Power_10times; }

        /**
         * @brief 获取超级电容剩余能量
         * @return 剩余能量值(J)
         */
        float GetCurrentEnergy() { return CurrentEnergy; }
        
        /**
         * @brief 获取电机+放电功率
         * @return 电机功率值(W)
         */
        float GetPower() { return Power; }
    private:
        float Power_10times;    // 10倍电管功率
        float CurrentEnergy;    // 超电剩余能量
        float Power;            // 电机+放电功率
        uint8_t state;          // 状态 warning，放电；error：某种错误，normal：正常
        uint8_t cmd;            // 当前命令

        uint32_t id;            // 超级电容ID
        float RatedPower;       // 等级功率
        uint8_t Instruction;    // 超电指令 0:开启 1:关闭
        float BufferEnergy;     // 缓冲能量
        uint32_t last_update_time; // 上次更新时间
        bool isSupercapOnline;
        bool isRefereeOnline;
};

extern uint8_t BoardRx[23];
extern Supercapacitor supercap; // 超电类声明

#endif
