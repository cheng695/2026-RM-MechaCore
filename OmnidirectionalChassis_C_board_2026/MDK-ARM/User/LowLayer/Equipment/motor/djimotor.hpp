#ifndef _DJIMOTOR_HPP_
#define _DJIMOTOR_HPP_

#include "../User/LowLayer/Equipment/motor/motor_base.hpp"

namespace motor
{
    struct Coefficient 
    {
        double reduction_ratio;      // 减速比
        double torque_constant;      // 力矩常数 (Nm/A)
        double feedback_current_max; // 反馈最大电流 (A)
        double current_max;          // 最大电流 (A)
        double encoder_resolution;   // 编码器分辨率
        double max_angle;            // 最大角度

        // 自动计算的参数
        double encoder_to_deg;                  // 编码器值转角度系数
        double encoder_to_rpm;                  // 编码器值转速度系数
        double rpm_to_radps;                    // RPM转角速度系数
        double current_to_torque_coefficient;   // 电流转扭矩系数
        double feedback_to_current_coefficient; // 反馈电流转电流系数
        double deg_to_real;                     // 角度转实际角度系数

        static constexpr double deg_to_rad = 0.017453292519611;
        static constexpr double rad_to_deg = 1 / 0.017453292519611;
            
        // 构造函数带参数计算 rr减速比，tc扭矩常数，fmc反馈最大电流，mc最大电流，er编码器分辨率 ma最大角度（一圈，叠加角用）
        Coefficient (double rr, double tc, double fmc, double mc, double er, double ma)
            : reduction_ratio(rr), torque_constant(tc), feedback_current_max(fmc), current_max(mc), 
            encoder_resolution(er), max_angle(ma)
        {
            constexpr double PI = 3.14159265358979323846;
            encoder_to_deg = 360.0 / encoder_resolution;
            rpm_to_radps = 1 / reduction_ratio / 60 * 2 * PI;
            encoder_to_rpm = 1 / reduction_ratio;
            current_to_torque_coefficient = reduction_ratio * torque_constant / feedback_current_max * current_max;
            feedback_to_current_coefficient = current_max / feedback_current_max;
            deg_to_real = 1 / reduction_ratio;
        }
    };

    template<uint8_t N> class DJImotorBase : public MotorBase<N>
    {
            public:
            
            struct Coefficient* CoefficientData; 
            
            DJImotorBase(uint16_t init_id_,  uint8_t(*rxid_)[N], uint32_t txid_, Coefficient* data)
            {
                for(uint8_t i = 0; i < N; ++i)
                {
                    rxid[i] = (*rxid_)[i];
                }
                txid = txid_;
                init_id = init_id_;
                CoefficientData = data;
                can_ = nullptr; // 保存CAN实例引用
            }

            bool Send0x200(int16_t m1, int16_t m2, int16_t m3, int16_t m4) //值都放入这条函数
            {
                return SendMotorData(0x200, m1, m2, m3, m4);
            }
            
            bool Send0x1FF(int16_t m1, int16_t m2, int16_t m3, int16_t m4) //值都放入这条函数
            {
                return SendMotorData(0x1FF, m1, m2, m3, m4);
            }

            bool Send0x1FE(int16_t m1, int16_t m2, int16_t m3, int16_t m4) //值都放入这条函数
            {
                return SendMotorData(0x1FE, m1, m2, m3, m4);
            }

            void PackInt16(uint8_t* dest, int16_t value)
            {
                dest[0] = (value >> 8) & 0xFF;
                dest[1] = value & 0xFF;
            }

            bool SendMotorData(uint16_t cmdId, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
            {
                CanDriver::CanFrame frame;
                frame.id = cmdId;
                frame.dlc = 8;
                frame.isExtended = false;
                frame.isRemote = false;
                
                // 打包4个电机的16位数据（每个电机占2个字节）
                PackInt16(&frame.data[0], m1);
                PackInt16(&frame.data[2], m2);
                PackInt16(&frame.data[4], m3);
                PackInt16(&frame.data[6], m4);
                
                // 通过CAN发送
                if(can_) return can_->Send(frame);
                return false;
            }

            void checkMotorsState()
            {
                for(uint8_t i = 0; i < N; i++)
                {
                    this->getTime();
                    this->checkTime(200);
                    this->encoderdata[i].isOnline = this->isOnline;
                    // 注意：这里我们不直接处理离线情况，而是在MotorState函数中检查isOnline标志
                }
            }

            void SetCan(CanDriver::CanHal* can){can_ = can;}

            void DataUpdate(const uint8_t *data, uint8_t index)
            {
                this->encoderdata[index].ref_angle          = (int16_t)(data[0]<<8|data[1]);     //角度值
                this->encoderdata[index].ref_speed          = (int16_t)(data[2]<<8|data[3]);     //速度
                this->encoderdata[index].ref_torqueCurrent  = (int16_t)(data[4]<<8|data[5]);     //电流
                this->encoderdata[index].ref_temperate      = data[6];                           //温度
            }

            void RmMotorRxData(const CanDriver::CanFrame& frame)
            {
                const uint16_t receive_id = frame.id;

                for(uint8_t i = 0; i < N; ++i)
                {
                    if(receive_id == init_id + rxid[i])
                    {
                        // 获取电机数据
                        DataUpdate(frame.data, i);
                        
                        Configure(i);
                    }
                }
            }

            void Configure(size_t i)
            {
                this->encoderdata[i].angle_deg       = this->encoderdata[i].ref_angle * CoefficientData->encoder_to_deg;
                this->encoderdata[i].angle_rad       = this->encoderdata[i].angle_deg * CoefficientData->deg_to_rad;
                this->encoderdata[i].speed_rpm       = this->encoderdata[i].ref_speed * CoefficientData->encoder_to_rpm;
                this->encoderdata[i].speed_rad_s     = this->encoderdata[i].speed_rpm * CoefficientData->rpm_to_radps;
                this->encoderdata[i].torque          = this->encoderdata[i].ref_torqueCurrent * CoefficientData->current_to_torque_coefficient;
                this->encoderdata[i].torquecurrent   = this->encoderdata[i].ref_torqueCurrent * CoefficientData->feedback_to_current_coefficient;

                this->encoderdata[i].angle_odometry_8191.update(static_cast<float>(this->encoderdata[i].ref_angle));
                this->encoderdata[i].angle_odometry_360.update(static_cast<float>(this->encoderdata[i].ref_angle));
                //this->encoderdata[i].angle_odometry_....update(static_cast<float>(this->encoderdata[i].ref_angle));

                this->encoderdata[i].add_angle = this->encoderdata[i].angle_odometry_8191.getAccumulatedAngle();

                this->encoderdata[i].time = DJImotorBase<N>::getlastTime();
            }


        private:
            uint8_t rxid[N];
            uint32_t txid;
            uint16_t init_id;
            CanDriver::CanHal* can_; // 添加CAN驱动实例指针
    };


    template <uint8_t N> class GM2006 : public DJImotorBase<N>
    {
    public:
        GM2006(uint16_t init_id_, uint8_t (*rxid_)[N], uint32_t txid_)
            : DJImotorBase<N>(init_id_, rxid_, txid_,
                            // 直接构造参数对象
                        new Coefficient(36.0, 0.18 / 36.0, 16384, 10, 8192, 360))
        {
        }
        ~GM2006() {
            delete this->CoefficientData;  // 释放内存
        }
    };

    template <uint8_t N> class GM3508 : public DJImotorBase<N>
    {
    public:
        GM3508(uint16_t init_id_, uint8_t (*rxid_)[N], uint32_t txid_)
            : DJImotorBase<N>(init_id_, rxid_, txid_,
                            // 直接构造参数对象
                        new Coefficient(1.0, 0.3 / 1.0, 16384, 20, 8192, 360))
        {
        }
        ~GM3508() {
            delete this->CoefficientData;  // 释放内存
        }
    };

    template <uint8_t N> class GM6020 : public DJImotorBase<N>
    {
    public:
        GM6020(uint16_t init_id_, uint8_t (*rxid_)[N], uint32_t txid_)
            : DJImotorBase<N>(init_id_, rxid_, txid_,
                            // 直接构造参数对象
                        new Coefficient(1.0, 0.7 * 1.0, 16384, 3, 8192, 360))
        {
        }
        ~GM6020() {
            delete this->CoefficientData;  // 释放内存
        }

    };

}

#ifdef __cplusplus
extern "C" {
#endif
	
void CanInit();

#ifdef __cplusplus
}
#endif

#endif
