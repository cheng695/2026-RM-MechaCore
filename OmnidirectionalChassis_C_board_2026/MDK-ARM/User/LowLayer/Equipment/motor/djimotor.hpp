#ifndef _DJIMOTOR_HPP_
#define _DJIMOTOR_HPP_

#include "../User/LowLayer/HAL_/can/can_driver.hpp"



// namespace motor
// {
//     struct AngleCalculator 
//     {
//         float maxAngle;           // 最大角度值
//         float accumulatedAngle;   // 累计角度
//         float lastAngleRaw;       // 上一次原始角度值
//         bool initialized;         // 初始化标志
        
//         AngleCalculator(float maxAngleValue) : 
//             maxAngle(maxAngleValue), 
//             accumulatedAngle(0.0f), 
//             lastAngleRaw(0.0f), 
//             initialized(false) {}
        
//         // 更新累计角度
//         float update(float currentRawAngle) 
//         {
//             if (!initialized) 
//             {
//                 lastAngleRaw = currentRawAngle;
//                 accumulatedAngle = currentRawAngle;
//                 initialized = true;
//                 return accumulatedAngle;
//             }
            
//             // 处理角度翻转情况
//             float delta = currentRawAngle - lastAngleRaw;
//             const float halfMax = maxAngle / 2.0f;
            
//             if (delta > halfMax) 
//             {
//                 // 向下翻转
//                 accumulatedAngle += (-maxAngle + delta);
//             } 
//             else if (delta < -halfMax) 
//             {
//                 // 向上翻转
//                 accumulatedAngle += (maxAngle + delta);
//             }
//             else 
//             {
//                 // 正常变化
//                 accumulatedAngle += delta;
//             }
            
//             lastAngleRaw = currentRawAngle;
//             return accumulatedAngle;
//         }
//     };

//     class DJImotor : public State::monitoring
//     {   
//         public:
//             DJImotor(uint32_t recv_id) { recv_id_ = recv_id; };
//             void DataUpdate(const uint8_t *data, float Maximum_angle);

//             float getAccumulatedAngle(float maxAngleValue) 
//             {
//                 // 查找是否存在对应的角度计算器
//                 for (auto& calc : angleCalculators) 
//                 {
//                     if (calc.maxAngle == maxAngleValue) 
//                     {
//                         return calc.update((float)angle);
//                     }
//                 }
                
//                 // 如果不存在，则创建新的角度计算器
//                 angleCalculators.emplace_back(maxAngleValue);
//                 return angleCalculators.back().update((float)angle);
//             }

//             float totalAngle;               //累计角度
//             float MotorSet;                 //电机设置值
        
//         private:
//             uint32_t recv_id_;              //接收id
//             int16_t ref_angle;              //转子机械角度  0-8191
//             int16_t ref_speed;              //转子速度      rpm
//             int16_t ref_torqueCurrent;      //实际扭矩电流
//             uint8_t ref_temperate;          //电机温度      摄氏度

//             float angle;                    //转子机械角度,float              
//             float speed;                    //转子速度,float        
//             float torquecurrent;            //实际扭矩电流,float
//             float torque;                   //电机扭矩

//             float angle_8191;               //转子机械角度  0-8191
//             float angle_360;                //转子机械角度  0-360
//             float speed_rad;                //转子速度      rad/s

//             //提供两个接口，这个是最大角度的值，前面的是结构体，里面有一些关于累加角度的变量和函数
//             AngleCalculator angle8191Calc{8191.0f};
//             AngleCalculator angle360Calc {360.0f };
//             std::vector<AngleCalculator> angleCalculators;
//     };

//     class DJIMotorController 
//     { 
//     public:
//         explicit DJIMotorController(CanDriver::CanHal& can) : can_(can) {}
    
//         // 更清晰的API命名
//         bool Send0x200(int16_t m1, int16_t m2, int16_t m3, int16_t m4) //值都放入这条函数
//         {
//             return SendMotorData(0x200, m1, m2, m3, m4);
//         }
        
//         bool Send0x1FF(int16_t m1, int16_t m2, int16_t m3, int16_t m4) //值都放入这条函数
//         {
//             return SendMotorData(0x1FF, m1, m2, m3, m4);
//         }

//         bool Send0x1FE(int16_t m1, int16_t m2, int16_t m3, int16_t m4) //值都放入这条函数
//         {
//             return SendMotorData(0x1FE, m1, m2, m3, m4);
//         }

//     private:
//         bool SendMotorData(uint16_t cmdId, int16_t m1, int16_t m2, int16_t m3, int16_t m4); //发送帧处理函数
//         CanDriver::CanHal& can_;
        
//         static void PackInt16(uint8_t* dst, int16_t val) //数据处理函数
//         {
//             dst[0] = static_cast<uint8_t>(val >> 8);
//             dst[1] = static_cast<uint8_t>(val & 0xFF);
//         };
//     };
// }

// void RmMotorRxData(const CanDriver::CanFrame& frame);
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

    template<uint8_t N> class DJImotorBase : public motor_base<N>
    {
        public:
            DJImotorBase(uint16_t init_id_, const uint8_t(*rxid_)[N], uint32_t txid_, Coefficient* data)
            {
                for(uint8_t i = 0; i < N; ++i)
                {
                    rxid[i] = rxid_[i];
                }
                txid = txid_;
                init_id = init_id_;
                CoefficientData = data;
            }

            Coefficient CoefficientData;

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

                    }
                }
            }

            void Configure(size_t i)
            {
                this->encoderdata[i].angle_deg = this->encoderdata[i].ref_angle * CoefficientData->encoder_to_deg;
                this->encoderdata[i].angle_rad = this->encoderdata[i].angle_deg * CoefficientData->deg_to_rad;
                this->encoderdata[i].speed_rpm = this->encoderdata[i].ref_speed * CoefficientData->encoder_to_rpm;
                this->encoderdata[i].speed_rad = this->encoderdata[i].speed_rpm * CoefficientData->rpm_to_radps;
                this->encoderdata[i].torque_Nm = this->encoderdata[i].ref_torqueCurrent * CoefficientData->current_to_torque_coefficient;
                this->encoderdata[i].current_A = this->encoderdata[i].ref_torqueCurrent * CoefficientData->feedback_to_current_coefficient;
                this->encoderdata[i].temperate_C = this->encoderdata[i].ref_temperate;

                this->encoderdata[i].add_angle = 
                
            }


        private:
            uint8_t rxid[N];
            uint32_t txid;
            uint16_t init_id


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
