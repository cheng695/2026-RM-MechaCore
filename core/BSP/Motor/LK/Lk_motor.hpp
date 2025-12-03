#ifndef LK_MOTOR_HPP
#define LK_MOTOR_HPP

#pragma once
//fix：修复上传错误
#include "../BSP/Motor/MotorBase.hpp"
#include "core/BSP/Common/StateWatch/state_watch.hpp"
#include "../HAL/CAN/can_hal.hpp"
#define PI 3.14159265358979323846
namespace BSP::Motor::LK
{

namespace BSP::Motor::LK
{
    // 参数结构体定义
    struct Parameters
    {
        double reduction_ratio;      // 减速比
        double torque_constant;      // 力矩常数 (Nm/A)
        double feedback_current_max; // 反馈最大电流 (A)
        double current_max;          // 最大电流 (A)
        double encoder_resolution;   // 编码器分辨率

        // 自动计算的参数
        double encoder_to_deg; // 编码器值转角度系数
        double encoder_to_rpm;
        double rpm_to_radps;                    // RPM转角速度系数
        double current_to_torque_coefficient;   // 电流转扭矩系数   
        double feedback_to_current_coefficient; // 反馈电流转电流系数
        double deg_to_real;                     // 角度转实际角度系数

        static constexpr double deg_to_rad = 0.017453292519611;
        static constexpr double rad_to_deg = 1 / 0.017453292519611;

        // 构造函数带参数计算
        Parameters(double rr, double tc, double fmc, double mc, double er)
            : reduction_ratio(rr), torque_constant(tc), feedback_current_max(fmc), 
              current_max(mc), encoder_resolution(er)
        {
            encoder_to_deg = 360.0 / encoder_resolution;
            rpm_to_radps = 1 / reduction_ratio / 60 * 2 * 3.14159265358979323846;
            encoder_to_rpm = 1 / reduction_ratio;
            current_to_torque_coefficient = reduction_ratio * torque_constant / feedback_current_max * current_max;
            feedback_to_current_coefficient = current_max / feedback_current_max;
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
                
                // 初始化多圈角度数据
                multi_angle_data_[i].total_angle = 0.0;
                multi_angle_data_[i].last_angle = 0.0;
                multi_angle_data_[i].allow_accumulate = false;
                multi_angle_data_[i].is_initialized = false;
            }
        }

    private:
        void Configure(size_t i)
        {
            const auto &params = params_;

            this->unit_data_[i].angle_Deg = feedback_[i].angle * params.encoder_to_deg;
            this->unit_data_[i].angle_Rad = this->unit_data_[i].angle_Deg * params.deg_to_rad;
            this->unit_data_[i].velocity_Rad = feedback_[i].velocity * params.rpm_to_radps;
            this->unit_data_[i].velocity_Rpm = feedback_[i].velocity * params.encoder_to_rpm;
            this->unit_data_[i].current_A = feedback_[i].current * params.feedback_to_current_coefficient;
            this->unit_data_[i].torque_Nm = feedback_[i].current * params.current_to_torque_coefficient;
            this->unit_data_[i].temperature_C = feedback_[i].temperature;

            // 多圈角度计算
            if (multi_angle_data_[i].allow_accumulate) 
            {
                if (!multi_angle_data_[i].is_initialized)
                {
                    multi_angle_data_[i].last_angle = this->unit_data_[i].angle_Deg;
                    multi_angle_data_[i].is_initialized = true;
                }
                else
                {
                    double last_angle = multi_angle_data_[i].last_angle;
                    double delta = this->unit_data_[i].angle_Deg - last_angle;
                    
                    // 处理360°跳变
                    if (delta > 180.0) 
                        delta -= 360.0;
                    else if (delta < -180.0) 
                        delta += 360.0;
                    
                    multi_angle_data_[i].total_angle += delta;
                    this->unit_data_[i].add_angle = delta;
                }
            }
            
            multi_angle_data_[i].last_angle = this->unit_data_[i].angle_Deg;
            this->unit_data_[i].last_angle = this->unit_data_[i].angle_Deg;
        }

    public:
        /**
     * @brief 设置发送数据
     * 
     * @param data 数据发送的数据
     * @param id 电机ID
     */
    void setCAN(int16_t data, int id)
    {
        // LK电机发送格式与DJI不同，需要根据具体命令调整
        // 这里先使用类似DJI的格式
        msd[(id - 1) * 2] = data >> 8;
        msd[(id - 1) * 2 + 1] = data << 8 >> 8;
    }

    /**
     * @brief 发送Can数据
     * 
     * @param han Can句柄
     * @param pTxMailbox 邮箱
     */
    void sendCAN(uint8_t id, uint8_t pTxMailbox)
    {
        this->send_can_frame(send_idxs_[id - 1], msd, 8, pTxMailbox);
    }
    /**
     * @brief 使能电机
     * 
     * @param hcan CAN句柄
     */
    void ON(uint8_t id)
    {
        uint8_t data[8] = {0x88};
        this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 失能电机
     * 
     * @param hcan CAN句柄
     */
    void OFF(uint8_t id)
    {
        uint8_t data[8] = {0x81};
        this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 清除错误
     * 
     * @param hcan CAN句柄
     */
    void clear_err(uint8_t id)
    {
        uint8_t data[8] = {0x9B};
        this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 位置控制
     * 
     * @param hcan CAN句柄
     * @param angle 目标角度（度）
     * @param speed 速度限制（RPM）
     * @param id 电机ID
     */
    void SetPositionCtrl(uint8_t id, int32_t angle, uint16_t speed)
    {
        uint8_t data[8];
        uint32_t encoder_value = angle * 100; // 根据实际转换关系调整
        
        data[0] = 0xA4;
        data[1] = 0x00;
        data[2] = speed & 0xFF;
        data[3] = (speed >> 8) & 0xFF;
        data[4] = encoder_value & 0xFF;
        data[5] = (encoder_value >> 8) & 0xFF;
        data[6] = (encoder_value >> 16) & 0xFF;
        data[7] = (encoder_value >> 24) & 0xFF;

        this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 扭矩控制
     * 
     * @param hcan CAN句柄
     * @param torque 目标扭矩
     * @param id 电机ID
     */
    void SetTorqueCtrl(uint8_t id, int16_t torque)
    {
        if (torque > 2048) torque = 2048;
        if (torque < -2048) torque = -2048;
        uint8_t data[8];
        
        data[0] = 0xA1;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        data[4] = torque & 0xFF;
        data[5] = (torque >> 8) & 0xFF;
        data[6] = 0x00;
        data[7] = 0x00;

       this->send_can_frame(init_address + send_idxs_[id - 1], data, 8);
    }

    /**
     * @brief 获取多圈角度
     * 
     * @param id 电机ID
     * @return float 多圈角度（度）
     */
    float getMultiAngle(uint8_t id)
    {
        return multi_angle_data_[id].total_angle;
    }

        /**
         * @brief LK电机的位置控制方法
         */
        void ctrl_Position(CAN_HandleTypeDef *hcan, uint8_t motor_index, int32_t angle, uint16_t speed)
        {
            if (motor_index < 1 || motor_index > N) return;

            uint8_t data[8];
            uint32_t encoder_value = angle * 100; // 根据实际转换关系调整
            
            data[0] = 0xA4;
            data[1] = 0x00;
            data[2] = speed & 0xFF;
            data[3] = (speed >> 8) & 0xFF;
            data[4] = encoder_value & 0xFF;
            data[5] = (encoder_value >> 8) & 0xFF;
            data[6] = (encoder_value >> 16) & 0xFF;
            data[7] = (encoder_value >> 24) & 0xFF;

            CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], data, CAN_TX_MAILBOX1);
        }

        /**
         * @brief LK电机的扭矩控制方法
         */
        void ctrl_Torque(CAN_HandleTypeDef *hcan, uint8_t motor_index, int16_t torque)
        {
            if (motor_index < 1 || motor_index > N) return;

            // 扭矩限制
            if (torque > 2048) torque = 2048;
            if (torque < -2048) torque = -2048;
            
            uint8_t data[8];
            data[0] = 0xA1;
            data[1] = 0x00;
            data[2] = 0x00;
            data[3] = 0x00;
            data[4] = torque & 0xFF;
            data[5] = (torque >> 8) & 0xFF;
            data[6] = 0x00;
            data[7] = 0x00;

            CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], data, CAN_TX_MAILBOX1);
        }

        /**
         * @brief 使能LK电机
         */
        void On(CAN_HandleTypeDef *hcan, uint8_t motor_index)
        {
            if (motor_index < 1 || motor_index > N) return;

            uint8_t data[8] = {0x88};
            CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], data, CAN_TX_MAILBOX1);
        }

        /**
         * @brief 失能LK电机
         */
        void Off(CAN_HandleTypeDef *hcan, uint8_t motor_index)
        {
            if (motor_index < 1 || motor_index > N) return;

            uint8_t data[8] = {0x81};
            CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], data, CAN_TX_MAILBOX1);
        }

        /**
         * @brief 清除LK电机错误
         */
        void ClearErr(CAN_HandleTypeDef *hcan, uint8_t motor_index)
        {
            if (motor_index < 1 || motor_index > N) return;

            uint8_t data[8] = {0x9B};
            CAN::BSP::Can_Send(hcan, init_address + send_idxs_[motor_index - 1], data, CAN_TX_MAILBOX1);
        }

        /**
         * @brief 获取多圈角度
         */
        float getMultiAngle(uint8_t id)
        {
            if (id < 1 || id > N) return 0.0f;
            return multi_angle_data_[id - 1].total_angle;
        }

        /**
         * @brief 设置是否允许累计多圈角度
         */
        void setAllowAccumulate(uint8_t id, bool allow)
        {
            if (id < 1 || id > N) return;
            multi_angle_data_[id - 1].allow_accumulate = allow;
        }

        /**
         * @brief 获取是否允许累计多圈角度
         */
        bool getAllowAccumulate(uint8_t id)
        {
            if (id < 1 || id > N) return false;
            return multi_angle_data_[id - 1].allow_accumulate;
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
     * @brief LK4005电机类
     */
    template <uint8_t N> 
    class LK4005 : public LkMotorBase<N>
    {
    public:
        LK4005(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
            : LkMotorBase<N>(Init_id, ids, send_idxs,
                            Parameters(10.0,     // 减速比
                                     0.06,      // 扭矩常数
                                     4096,      // 最大反馈电流
                                     2.7,       // 最大电流 
                                     65536.0))  // 编码器分辨率
        {
        }
    };

} // namespace BSP::Motor::LK

#endif