#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "buzzer_manager.hpp"
#include "can_hal.hpp"
#include "DT7.hpp"
#include "state_watch.hpp"
#include "uart_hal.hpp"
#include <cstring>
#include "RM_RefereeSystem.h"

// ──── 板间通讯协议 ──────────────────────────────────────────────────
/**
 * @brief 板间通讯�?
 * @details 负责处理与上位机板子之间的串口通讯，包括连接状态监测、数据解析等功能
 */
class BoardCommunication
{
    public:
        #pragma pack(1)
        // 发送：底盘→云�?上行数据�?
        struct BoardTX
        {
            uint16_t shooter_barrel_cooling_value;  // 2  枪口每秒冷却�?
            uint16_t shooter_barrel_heat_limit;     // 2  枪口热量上限
            bool     chassis_spin;                  // 1  �����Ƿ�����С����
        };
        
        // 接收：云台→底盘 下行数据�?
        struct BoardRX
        {
            uint8_t  rc_data[18];   // 18  遥控器数据转�?(DT7Rx_buffer)
            float    angle;         // 4   Motor6020 角度 (rad)
            bool     scroll;        // 1   发射机构 scroll 状�?
        };
        #pragma pack()

        BoardTX board_tx_;
        BoardRX board_rx_;

    public:
        /**
         * @brief 构造函�?
         * @param timeThreshold 连接超时阈�?毫秒)，默�?00ms
         */
        BoardCommunication(int timeThreshold = 100) : statewatch_(timeThreshold) 
        {
        }

        /**
         * @brief 虚析构函�?
         */
        virtual ~BoardCommunication() = default;

        /**
         * @brief 更新时间�?
         * @details 记录最后一次通讯时间，用于判断连接状�?
         */
        void updateTimestamp()
        {
            statewatch_.UpdateLastTime();
        }

        /**
         * @brief 检查连接状�?
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
        
        void SetYawAngle(const uint8_t* data)
        {
            std::memcpy(&board_rx_.angle, data, sizeof(float));
        }

        void SetScroll(const uint8_t* data)
        {
            std::memcpy(&board_rx_.scroll, data, sizeof(bool));
        }

        float GetYawAngle() { return board_rx_.angle; } // 云台角度(弧度) 原始�?
        bool GetScroll() { return board_rx_.scroll; } // 能否小陀�?true: 不能小陀�?false: 能小陀螺（遥控器模式下�?

    private:
        BSP::WATCH_STATE::StateWatch statewatch_;  // 连接状态监视器
};

/**
 * @brief 超级电容控制�?
 * @details 负责超级电容的CAN通讯控制，包括状态读取和指令下发
 */
class Supercapacitor
{
    public:
        /**
         * @brief 构造函�?
         * @param id_ 超级电容CAN ID
         */
        Supercapacitor(uint32_t id_)
        {
            id = id_;
            last_update_time = 0;
        } 

        /**
         * @brief 检查超级电容是否在�?
         * @param timeout_ms 超时时间(ms)
         * @return true 在线
         */
        bool isConnected(uint32_t timeout_ms = 100)
        {
            return (HAL_GetTick() - last_update_time) < timeout_ms;
        } 

        /**
         * @brief 解析CAN接收数据
         * @param frame CAN数据�?
         * @details 解析来自超级电容的状态反馈数�?
         */
        void Parse(const HAL::CAN::Frame &frame)
        {
            if (frame.id == 0x777)
            {
                const uint8_t* pData = frame.data;
                    
                Power_10times = (float)((int16_t)((pData[0] << 8) | pData[1]));  // 电管功率(10�?
                CurrentEnergy = (float)((int16_t)((pData[2] << 8) | pData[3]));  // 剩余能量
                Power         = (float)((int16_t)((pData[4] << 8) | pData[5]));  // 电机+放电（正的）功率
                state         = pData[6];                                        // 工作状�?
                cmd           = pData[7];                                        // 当前指令

                Power_10times /= 10;    // 转换为实际电管功率�?      
                Power /= -10;           // 转换为实际电�?放电（正的）功率 
                last_update_time = HAL_GetTick();
            }
        }

        /**
         * @brief 发送数据给超级电容
         * @param RatedPower 等级功率
         * @param Instruction 超电指令 0:开�?1:关闭
         * @param BufferEnergy 缓冲能量
         */
        void sendCAN()
        {
            uint8_t send_data[8] = {0};
            int16_t power_int = (int16_t)(RatedPower);
            send_data[0] = (power_int >> 8) & 0xFF;  // 等级功率高字�?
            send_data[1] = power_int & 0xFF;         // 等级功率低字�?

            send_data[2] = Instruction;              // 控制指令

            int16_t buffer_int = (int16_t)BufferEnergy;
            send_data[3] = (buffer_int >> 8) & 0xFF; // 缓冲能量高字�?
            send_data[4] = buffer_int & 0xFF;        // 缓冲能量低字�?

            send_data[5] = isSupercapOnline;  // 超电连接标志
            send_data[6] = isRefereeOnline;   // 裁判系统连接标志
            send_data[7] = 0;  // 保留字节
            
            HAL::CAN::Frame frame;
            frame.id = id;                    // CAN ID
            frame.dlc = 8;                    // 数据长度
            memcpy(frame.data, send_data, sizeof(send_data));  // 拷贝数据
            frame.is_extended_id = false;     // 标准帧格�?
            frame.is_remote_frame = false;    // 数据�?
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);  // 发送CAN�?
        }

        /**
         * @brief 设置等级功率
         * @param power 等级功率�?W)
         */
        void setRatedPower(float power)
        {
            RatedPower = power;
        }

        /**
         * @brief 设置超电指令
         * @param instruction 超电指令 0:开�?1:关闭
         */
        void setInstruction(uint8_t instruction)
        {
            Instruction = instruction;
        }

        /**
         * @brief 设置缓冲能量
         * @param energy 缓冲能量�?J)
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
         * @return 功率�?W)
         */
        float GetPower_10times() { return Power_10times; }

        /**
         * @brief 获取超级电容剩余能量
         * @return 剩余能量�?J)
         */
        float GetCurrentEnergy() { return CurrentEnergy; }
        
        /**
         * @brief 获取电机+放电功率
         * @return 电机功率�?W)
         */
        float GetPower() { return Power; }
    private:
        float Power_10times;    // 10倍电管功�?
        float CurrentEnergy;    // 超电剩余能量
        float Power;            // 电机+放电功率
        uint8_t state;          // 状�?warning，放电；error：某种错误，normal：正�?
        uint8_t cmd;            // 当前命令

        uint32_t id;            // 超级电容ID
        float RatedPower;       // 等级功率
        uint8_t Instruction;    // 超电指令 0:开�?1:关闭
        float BufferEnergy;     // 缓冲能量
        uint32_t last_update_time; // 上次更新时间
        bool isSupercapOnline;
        bool isRefereeOnline;
};

extern uint8_t BoardRx[23];

#endif
