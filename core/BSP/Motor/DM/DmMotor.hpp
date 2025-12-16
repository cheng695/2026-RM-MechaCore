#pragma once
#include "../user/core/BSP/Motor/MotorBase.hpp"
#include "../user/core/HAL/CAN/can_hal.hpp"

namespace BSP::Motor::DM
{
    enum Model
    {
        MIT = 0,
        ANGLEVELOCITY = 1,
        VELOCITY = 2
    };

    // 参数结构体定义
    struct Parameters
    {
    }
};

/**
 * @brief 达妙电机的基类
 *
 * @tparam N 电机总数
 */
template <uint8_t N> class DMMotorBase : public MotorBase<N>
{
  protected:
    /**
     * @brief Construct a new Dji Motor Base object
     *
     * @param can_id can的初始id 比如3508与2006就是0x200
     * @param params 初始化转换国际单位的参数
     */
    DMMotorBase(uint16_t Init_id, const uint8_t (&recv_ids)[N], const uint32_t (&send_ids)[N], Parameters params)
        : init_address(Init_id), params_(params)
    {
        for (uint8_t i = 0; i < N; ++i)
        {
            recv_idxs_[i] = recv_ids[i]; // 接收ID索引
            send_idxs_[i] = send_ids[i]; // 发送ID存储
        }
    }

    // 构造函数带参数计算
    /**
     * @brief Construct a new Parameters object
     *
     * @param pmin 位置 最小值
     * @param pmax 位置 最大值
     * @param vmin 速度 最小值
     * @param vmax 速度 最大值
     * @param tmin 力矩 最小值
     * @param tmax 力矩 最大值
     * @param kpmin Kp 最小值
     * @param kpmax Kp 最大值
     * @param kdmin Kd 最小值
     * @param kdmax Kd 最大值
     */
    Parameters CreateParams(float pmin, float pmax, float vmin, float vmax, float tmin, float tmax, float kpmin,
                            float kpmax, float kdmin, float kdmax) const
    {
        return Parameters(pmin, pmax, vmin, vmax, tmin, tmax, kpmin, kpmax, kdmin, kdmax);
    }

  private:
    float uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    int float_to_uint(float x, float x_min, float x_max, int bits)
    {
        /// Converts a float to an unsigned int, given range and number of bits///
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    /**
     * @brief 将反馈数据转换为国际单位
     *
     * @param i 存结构体的id号
     */
    void Configure(size_t i)
    {
        const auto &params = params_;

        this->unit_data_[i].angle_Rad = uint_to_float(feedback_[i].angle, params.P_MIN, params.P_MAX, 16);

        this->unit_data_[i].velocity_Rad = uint_to_float(feedback_[i].velocity, params.V_MIN, params.V_MAX, 12);

        this->unit_data_[i].torque_Nm = uint_to_float(feedback_[i].torque, params.T_MIN, params.T_MAX, 12);

        this->unit_data_[i].temperature_C = feedback_[i].T_Mos;

        this->unit_data_[i].angle_Deg = this->unit_data_[i].angle_Rad * params_.rad_to_deg;

        double lastData = this->unit_data_[i].last_angle;
        double Data = this->unit_data_[i].angle_Deg;

        /**
         * @brief DM电机的MIT控制方法
         */
        void ctrl_Mit(uint8_t id, float _pos, float _vel, 
                float _KP, float _KD, float _torq)
        {
            uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
            pos_tmp = float_to_uint(_pos, params_.P_MIN, params_.P_MAX, 16);
            vel_tmp = float_to_uint(_vel, params_.V_MIN, params_.V_MAX, 12);
            kp_tmp = float_to_uint(_KP, params_.KP_MIN, params_.KP_MAX, 12);
            kd_tmp = float_to_uint(_KD, params_.KD_MIN, params_.KD_MAX, 12);
            tor_tmp = float_to_uint(_torq, params_.T_MIN, params_.T_MAX, 12);

            uint8_t send_data[8];
            send_data[0] = (pos_tmp >> 8);
            send_data[1] = (pos_tmp);
            send_data[2] = (vel_tmp >> 4);
            send_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
            send_data[4] = kp_tmp;
            send_data[5] = (kd_tmp >> 4);
            send_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
            send_data[7] = tor_tmp;

            HAL::CAN::Frame frame;
            frame.id = send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }

        this->unit_data_[i].last_angle = Data;

        /**
         * @brief DM电机的角度速度控制方法
         */
        void ctrl_AngleVelocity(uint8_t id, float _pos, float _vel)
        {
            uint8_t data[8];
            uint8_t *pbuf, *vbuf;

                Configure(i);

                break;
            }

            HAL::CAN::Frame frame;
            frame.id = 0X100 + send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, data, sizeof(data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }
    }

        /**
         * @brief DM电机的速度控制方法
         */
        void ctrl_Velocity(uint8_t id, float _vel)
        {
            uint8_t data[8] = {0};
            uint8_t *vbuf = (uint8_t*)&_vel;

        this->send_can_frame(init_address + send_idxs_[motor_index - 1], &posvel, 8, CAN_TX_MAILBOX1);
    }   
    

            HAL::CAN::Frame frame;
            frame.id = 0X200 + send_idxs_[id - 1];
            frame.dlc = 8;
            memcpy(frame.data, data, sizeof(data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }

        this->send_can_frame(init_address + send_idxs_[motor_index - 1], &vel, 8, CAN_TX_MAILBOX1);
    }
    /**
     * @brief 使能DM电机
     *
     * @param hcan 电机的can句柄
     * @param motor_index 电机序号从1开始
     */
    void On(uint8_t motor_index)
    {
        *(uint64_t *)(&send_data[0]) = 0xFCFFFFFFFFFFFFFF;

        /**
         * @brief 使能DM电机
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void On(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
            
            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }
        
        /**
         * @brief 失能DM电机
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void Off(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }

        /**
         * @brief 清除DM电机错误
         * @param mod 模式可以有3种: MIT = 0, ANGLEVELOCITY = 1, VELOCITY = 2
         */
        void ClearErr(uint8_t id, Model mod)
        {
            uint8_t send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

            HAL::CAN::Frame frame;
            if(mod == Model::MIT)
            {
                frame.id = send_idxs_[id - 1];
            }
            else if(mod == Model::ANGLEVELOCITY)
            {
                frame.id = 0x100 + send_idxs_[id - 1];
            }
            else if(mod == Model::VELOCITY)
            {
                frame.id = 0x200 + send_idxs_[id - 1];
            }
            frame.dlc = 8;
            memcpy(frame.data, send_data, sizeof(send_data));
            frame.is_extended_id = false;
            frame.is_remote_frame = false;
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);
        }
        return (this->state_watch_[index].getStatus() == BSP::WATCH_STATE::Status::ONLINE);
    }
  protected:
    struct alignas(uint64_t) DMMotorfeedback
    {
        uint8_t id : 4;
        uint8_t err : 4;
        uint16_t angle;
        uint16_t velocity : 12;
        uint16_t torque : 12;
        uint8_t T_Mos;
        uint8_t T_Rotor;
    };

    struct alignas(uint64_t) DM_VelPos
    {
        float pos_tmp;
        float vel_tmp;
    };

    struct alignas(uint32_t) DM_Vel
    {
        float vel_tmp;
    };

    DMMotorfeedback feedback_[N]; // 国际单位数据
    Parameters params_;           // 转国际单位参数列表
    uint8_t send_data[8];
};


    /**
     * @brief dji电机构造函数
     *
     * @param Init_id 初始ID
     * @param ids 电机ID列表
     * @param send_idxs_ 电机发送ID列表
     */
template <uint8_t N> class J4310 : public DMMotorBase<N>
{
  private:
    // // 定义参数生成方法
    // Parameters GetParameters() override
    // {
    //     return DMMotorBase<N>::CreateParams(-12.56, 12.56, -30, 30, -10, 10, 0.0, 500, 0.0, 5.0);
    // }

  public:
    // 子类构造时传递参数
    /**
     * @brief S2325电机类
     */
    J4310(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs_)[N])
        : DMMotorBase<N>(Init_id, ids, send_idxs_, Parameters(-12.56, 12.56, -30, 30, -3, 3, 0.0, 500, 0.0, 5.0))
    {
    public:
        J4310(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs)[N])
            : DMMotorBase<N>(Init_id, ids, send_idxs, 
                            Parameters(-12.56f, 12.56f, -45.0f, 45.0f, -18.0f, 18.0f, 0.0f, 500.0f, 0.0f, 5.0f))
        {
        }
    };

template <uint8_t N> class S2325 : public DMMotorBase<N>
{
  private:
    // // 定义参数生成方法
    // Parameters GetParameters() override
    // {
    //     return DMMotorBase<N>::CreateParams(-12.56, 12.56, -30, 30, -10, 10, 0.0, 500, 0.0, 5.0);
    // }

  public:
    // 子类构造时传递参数
    /**
     * @brief S2325电机类
     */
    S2325(uint16_t Init_id, const uint8_t (&ids)[N], const uint32_t (&send_idxs_)[N])
        : DMMotorBase<N>(Init_id, ids, send_idxs_, Parameters(-12.5, 12.5, -200, 200, -10, 10, 0.0, 500, 0.0, 5.0))
    {
    }
};
/**
 * @brief 创建实例时，模板填电机个数，构造函数共三个参数
 * 第一个是初始ID，
 * 第二个是电机接收ID列表
 * 第三个是电机发送ID列表
 */
inline J4310<1> Motor4310(0x00, {2}, {1});
inline S2325<2> Motor2325(0x00, {1,2}, {0x201,0x202});

} // namespace BSP::Motor::DM

