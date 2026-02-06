#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"
#include "../User/core/BSP/Common/StateWatch/state_watch.hpp"
#include "../User/core/BSP/Common/StateWatch/buzzer_manager.hpp"
#include "../User/core/HAL/CAN/can_hal.hpp"
#include "../User/core/APP/Referee/RM_RefereeSystem.h"

// 板间通讯接收缓冲区声明
extern uint8_t BoardRx[23];

/**
 * @brief 板间通讯类
 * @details 负责处理与上位机板子之间的串口通讯，包括连接状态监测、数据解析等功能
 */
class BoardCommunication
{
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
        
        /**
         * @brief 设置云台角度数据
         * @param data 指向角度数据的指针(4字节float)
         */
        void SetYawAngle(uint8_t *data)
        { 
            memcpy(&YawAngal, data, sizeof(float));
        }

        /**
         * @brief 设置能否小陀螺状态数据
         * @param data 指向能否小陀螺状态的指针(1字节bool)
         */
        void SetScroll(uint8_t *data)
        { 
            memcpy(&scroll, data, sizeof(bool));
        }

        /**
         * @brief 获取云台角度
         * @return 云台角度值(弧度单位)
         */
        float GetYawAngle() { return YawAngal; } //弧度

        /**
         * @brief 获取滚轮状态
         * @return 滚轮开关状态
         */
        bool GetScroll() { return scroll; }
    private:
        BSP::WATCH_STATE::StateWatch statewatch_;  // 连接状态监视器
        float YawAngal;                            // 云台角度(弧度)
        bool scroll;                               // 能否小陀螺 true: 不能小陀螺 false: 能小陀螺（遥控器模式下）
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

            send_data[5] = 0;  // 保留字节
            send_data[6] = 0;  // 保留字节
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
};

extern Supercapacitor supercap; // 超电类声明

#endif
