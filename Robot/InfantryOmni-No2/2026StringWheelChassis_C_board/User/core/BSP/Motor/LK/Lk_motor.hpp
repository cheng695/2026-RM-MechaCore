#ifndef LK_MOTOR_HPP
#define LK_MOTOR_HPP

#pragma once

#include "../user/core/BSP/Motor/MotorBase.hpp"
#include "../user/core/HAL/CAN/can_hal.hpp"

namespace BSP::Motor::LK
{
   // 参数结构体定义
   struct Parameters
   {
       double reduction_ratio;      // 减速比
       double torque_constant;      // 力矩常数 (Nm/A)
       double iq_resolution;        // 转矩电流分辨率 (A/LSB)，MF: 33/4096, MG: 66/4096
       double encoder_resolution;   // 编码器分辨率（14bit: 16384, 15bit: 32768, 16bit: 65536）

       // 自动计算的参数
       double encoder_to_deg;                  // 编码器值转角度系数
       double feedback_to_current_coefficient; // 反馈转矩电流转实际电流系数
       double deg_to_real;                     // 角度转实际角度系数（考虑减速比）

       static constexpr double deg_to_rad = 0.017453292519611;
       static constexpr double rad_to_deg = 1 / 0.017453292519611;

       // 构造函数带参数计算
       // rr: 减速比
       // tc: 力矩常数 (Nm/A)
       // iq_res: 转矩电流分辨率 (A/LSB)
       // er: 编码器分辨率
       Parameters(double rr, double tc, double iq_res, double er)
           : reduction_ratio(rr), torque_constant(tc), iq_resolution(iq_res), 
             encoder_resolution(er)
       {
           encoder_to_deg = 360.0 / encoder_resolution;
           feedback_to_current_coefficient = iq_resolution;
           deg_to_real = 1 / reduction_ratio;
       }
   };

   /**
    * @brief LK电机基类
    */
   template <uint8_t N> 
   class LkMotorBase : public MotorBase<N>
   {
   protected:
       struct alignas(uint64_t) LkMotorFeedback
       {
           uint8_t cmd;
           uint8_t temperature;
           int16_t current;
           int16_t velocity;
           uint16_t angle;
       };

       struct MultiAngleData
       {
           double total_angle;
           double last_angle;
           bool allow_accumulate;
           bool is_initialized;
       };

       /**
        * @brief 构造函数
        */
        LkMotorBase(uint16_t Init_id, const uint8_t (&recv_ids)[N], const uint32_t (&send_ids)[N], Parameters params)
            : init_address(Init_id), params_(params)
        {
            for (uint8_t i = 0; i < N; ++i)
            {
                recv_idxs_[i] = recv_ids[i];
                send_idxs_[i] = send_ids[i];
            }
        }

    private:
        void Configure(size_t i)
        {
            const auto &params = params_;

            // 编码器值转角度（16bit编码器：0~65535）
            this->unit_data_[i].angle_Deg = feedback_[i].angle * params.encoder_to_deg;
            this->unit_data_[i].angle_Rad = this->unit_data_[i].angle_Deg * params.deg_to_rad;
            
            // 速度：协议规定为 1dps/LSB（度/秒）
            double velocity_dps = feedback_[i].velocity;  // 单位：度/秒
            this->unit_data_[i].velocity_Rad = velocity_dps * params.deg_to_rad;  // 转换为 rad/s
            this->unit_data_[i].velocity_Rpm = velocity_dps / 6.0;  // 转换为 RPM (度/秒 ÷ 6 = RPM)
            
            // 转矩电流转换（根据电机型号不同，分辨率不同）
            this->unit_data_[i].current_A = feedback_[i].current * params.feedback_to_current_coefficient;
            this->unit_data_[i].torque_Nm = this->unit_data_[i].current_A * params.torque_constant * params.reduction_ratio;
            this->unit_data_[i].temperature_C = feedback_[i].temperature;

            // 多圈角度累加
            double lastData = this->unit_data_[i].last_angle;
            double Data = this->unit_data_[i].angle_Deg;

            if (Data - lastData < -180) // 正转
                this->unit_data_[i].add_angle += (360 - lastData + Data) * params.deg_to_real;
            else if (Data - lastData > 180) // 反转
                this->unit_data_[i].add_angle += -(360 - Data + lastData) * params.deg_to_real;
            else
                this->unit_data_[i].add_angle += (Data - lastData) * params.deg_to_real;

            this->unit_data_[i].last_angle = Data;
        }

        HAL::CAN::Frame msd;

    public:
        /**
            * @brief 解析CAN数据
            */
        void Parse(const HAL::CAN::Frame &frame) override
        {
            for (uint8_t i = 0; i < N; ++i)
            {
                if (frame.id == init_address + recv_idxs_[i])
                {
                    const uint8_t* pData = frame.data;
                        
                    feedback_[i].cmd = pData[0];
                    feedback_[i].temperature = pData[1];
                    feedback_[i].current = (int16_t)((pData[3] << 8) | pData[2]);
                    feedback_[i].velocity = (int16_t)((pData[5] << 8) | pData[4]);
                    feedback_[i].angle = (uint16_t)((pData[7] << 8) | pData[6]);

                    Configure(i);
                    this->updateTimestamp(i + 1);
                }
            }
        }

        /**
         * @brief               发送Can数据
         *
         * @param id            电机ID (1~N)
         */
        void sendCAN(uint8_t id)
        {
            // 修改此处以适应新的CAN接口
            HAL::CAN::Frame frame;
            frame.id = init_address + send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, msd.data, 8);
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can1().send(frame);
        }

       /**
        * @brief LK电机的位置控制方法（多圈位置闭环控制命令2）
        * @param id 电机ID (1~N)
        * @param angle_0_01deg 目标角度，单位 0.01度，36000 = 360度
        * @param speed_dps 最大速度限制，单位 度/秒
        */
        void ctrl_Position(uint8_t id, int32_t angle_0_01deg, uint16_t speed_dps)
        {
            msd.data[0] = 0xA4;
            msd.data[1] = 0x00;
            msd.data[2] = speed_dps & 0xFF;
            msd.data[3] = (speed_dps >> 8) & 0xFF;
            msd.data[4] = angle_0_01deg & 0xFF;
            msd.data[5] = (angle_0_01deg >> 8) & 0xFF;
            msd.data[6] = (angle_0_01deg >> 16) & 0xFF;
            msd.data[7] = (angle_0_01deg >> 24) & 0xFF;

            sendCAN(id);
        }

       /**
        * @brief LK电机的扭矩控制方法（转矩闭环控制命令）
        * @param id 电机ID (1~N)
        * @param iqControl 转矩电流控制值，范围 -2048~2048
        *                  MF电机: 对应 -16.5A~16.5A
        *                  MG电机: 对应 -33A~33A
        */
        void ctrl_Torque(uint8_t id, int16_t iqControl)
        {
            if (id < 1 || id > N) return;

            // 扭矩限制
            if (iqControl > 2048) iqControl = 2048;
            if (iqControl < -2048) iqControl = -2048;
                
            msd.data[0] = 0xA1;
            msd.data[1] = 0x00;
            msd.data[2] = 0x00;
            msd.data[3] = 0x00;
            msd.data[4] = iqControl & 0xFF;
            msd.data[5] = (iqControl >> 8) & 0xFF;
            msd.data[6] = 0x00;
            msd.data[7] = 0x00;

            sendCAN(id);
        }

       /**
        * @brief 使能LK电机（电机运行命令 0x88）
        * @param id 电机ID (1~N)
        */
        void On(uint8_t id)
        {
            if (id < 1 || id > N) return;

            memset(msd.data, 0, 8);
            msd.data[0] = 0x88;
                
            sendCAN(id);
        }

       /**
        * @brief 失能LK电机（电机停止命令 0x81）
        * @param id 电机ID (1~N)
        */
        void Off(uint8_t id)
        {
            if (id < 1 || id > N) return;

            memset(msd.data, 0, 8);
            msd.data[0] = 0x81;
            
            sendCAN(id);
        }

       /**
        * @brief 清除LK电机错误（清除电机错误标志命令 0x9B）
        * @param id 电机ID (1~N)
        */
        void ClearErr(uint8_t id)
        {
            if (id < 1 || id > N) return;

            memset(msd.data, 0, 8);
            msd.data[0] = 0x9B;
            
            sendCAN(id);
        }

       /**
        * @brief 读取电机状态2（读取电机状态2命令 0x9C）
        * @param id 电机ID (1~N)
        */
        void SetReply(uint8_t id)
        {
            if (id < 1 || id > N) return;
            
            memset(msd.data, 0, 8);
            msd.data[0] = 0x9C;
            
            sendCAN(id);
        }

       /**
        * @brief 设置是否允许累计多圈角度
        */
       void setAllowAccumulate(uint8_t id, bool allow)
       {
           if (id < 1 || id > N) return;
           multi_angle_data_[id - 1].allow_accumulate = allow;
       }

   protected:
       const uint16_t init_address;
       uint8_t recv_idxs_[N];
       uint32_t send_idxs_[N];
       LkMotorFeedback feedback_[N];
       Parameters params_;
       MultiAngleData multi_angle_data_[N];
   };

   /**
    * @brief MG4005E电机类（MG系列，双编码器）
    * MG4005E-i10v3 参数：
    * - 扭矩常数：0.06 N.m/A
    * - 减速比：10:1 (PG4210)
    * - 转矩电流分辨率：66/4096 A/LSB (MG系列)
    * - 编码器：18bit(电机侧) + 14bit(减速器侧) 磁编码器
    * - 额定扭矩：1 N.m
    * - 峰值扭矩：2.5 N.m
    * 
    * 注意：协议中返回的是电机侧编码器数据
    */
   template <uint8_t N> 
   class MG4005E : public LkMotorBase<N>
   {
   public:
       MG4005E(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
           : LkMotorBase<N>(Init_id, ids, send_idxs,
                           Parameters(10.0,              // 减速比 (PG4210: 1:10)
                                    0.06,               // 扭矩常数 (N.m/A)
                                    66.0 / 4096.0,      // 转矩电流分辨率 (MG系列)
                                    262144.0))          // 编码器分辨率 (18bit电机侧: 2^18 = 262144)
       {
       }
   };

   /**
    * @brief LK4005电机类（MF系列）
    * 根据瓴控协议：
    * - 转矩电流分辨率：33/4096 A/LSB
    * - 编码器分辨率：16bit (65536)
    * - 减速比：10:1
    */
   template <uint8_t N> 
   class LK4005 : public LkMotorBase<N>
   {
   public:
       LK4005(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
           : LkMotorBase<N>(Init_id, ids, send_idxs,
                           Parameters(10.0,              // 减速比
                                    0.06,               // 力矩常数 (Nm/A)
                                    33.0 / 4096.0,      // 转矩电流分辨率 (MF系列)
                                    65536.0))           // 编码器分辨率 (16bit)
       {
       }
   };

   /**
    * @brief LK电机MG系列基类（通用MG系列电机）
    * MG系列转矩电流分辨率：66/4096 A/LSB
    */
   template <uint8_t N> 
   class LK_MG : public LkMotorBase<N>
   {
   public:
       LK_MG(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N],
             double reduction_ratio, double torque_constant, double encoder_res)
           : LkMotorBase<N>(Init_id, ids, send_idxs,
                           Parameters(reduction_ratio,
                                    torque_constant,
                                    66.0 / 4096.0,      // 转矩电流分辨率 (MG系列)
                                    encoder_res))
       {
       }
   };

    /**
     * @brief LK/MG电机实例示例
     * 
     * MG4005E-i10v3 使用方法：
     * inline MG4005E<4> Motor4005(0x140, {1, 2, 3, 4}, {1, 2, 3, 4});
     * 
     * 参数说明：
     * - 0x140: LK/MG电机的CAN基地址（固定值）
     * - {1, 2, 3, 4}: 接收ID列表，实际CAN ID为 0x141, 0x142, 0x143, 0x144
     * - {1, 2, 3, 4}: 发送ID列表，实际CAN ID为 0x141, 0x142, 0x143, 0x144
     * 
     * 注意事项：
     * 1. MG4005E 是双编码器电机（18bit电机侧 + 14bit减速器侧）
     * 2. CAN协议返回的是电机侧18bit编码器数据
     * 3. 转矩电流分辨率：MG系列 66/4096 A/LSB
     * 4. 速度单位：1 dps/LSB（度/秒）
     */
    //inline MG4005E<4> Motor4005(0x140, {1, 2, 3, 4}, {1, 2, 3, 4});
} // namespace BSP::Motor::LK

#endif