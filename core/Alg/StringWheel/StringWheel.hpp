#pragma once

#include <math.h>
#include <cstdint>
#include <cstring> // 添加头文件
#include "../APP/Tools.hpp"
#include "../HAL/HAL.hpp"
#include "../BSP/Motor/Dji/DjiMotor.hpp"
#include "../BSP/Motor/Lk/Lk_motor.hpp"
#define M_PI 3.14159265358979323846

const float ANGLE_45_RAD = M_PI / 4;    // 45度弧度值
const float COS_45 = cosf(ANGLE_45_RAD); // 45度余弦值
const float SIN_45 = sinf(ANGLE_45_RAD); // 45度正弦值


namespace Alg::Chassis
{
    class ChassisBase
    {
    public:
        struct Parameters
        {
            float Vx = 0.0f;
            float Vy = 0.0f;
            float Vw = 0.0f;
            float Speed[4] = {0};
            float MaxSpeed = 0.0f;
            float Angle[4] = {0};
            float a = 0.0f; //底盘中心 X 轴距离的一半
            float b = 0.0f; //底盘中心 Y 轴距离的一半

            //运动学计算中间变量
            float tempvx[4] = {0};
            float tempvy[4] = {0};
            float scaled_vw = 0.0f;
            float scaled_vx = 0.0f;
            float scaled_vy = 0.0f;
    
            //动力学计算中间变量
            float tmp_force[4] = {0};
            float Target_Wheel_Current[4] = {0};
            float Target_Wheel_Omega[4] = {0};
        };
        ChassisBase() = default;
        virtual ~ChassisBase() = default;
        virtual void UpDate(float vx, float vy, float vw, float maxSpeed) = 0;
        
        const Parameters& GetParams() const { return params_; } //当对象是const或通过const引用访问时，仍可以获取参数
        Parameters& GetParams() { return params_; }
        
        void SetChassisDimensions(float a, float b) // 添加设置底盘尺寸的公共接口
        { 
            params_.a = a;
            params_.b = b;
            InitializeWheelPositions();
        }
        
        inline void SetMaxSpeed(float maxSpeed) 
        {
            params_.MaxSpeed = maxSpeed;
        }
        inline float GetX() const      // 获取x方向线速度
        { 
            return params_.Vx; 
        }
        inline float GetY() const       // 获取y方向线速度
        {
            return params_.Vy; 
        }
        inline float GetZ() const       // 获取绕z轴角速度
        {
            return params_.Vw; 
        }
        inline void SetX(float vx)      // 设置x方向线速度
        { 
            params_.Vx = vx; 
        }
        inline void SetY(float vy)      // 设置y方向线速度
        { 
            params_.Vy = vy; 
        }
        inline void SetZ(float vw)      // 设置绕z轴角速度
        { 
            params_.Vw = vw; 
        }
    protected:
        Parameters params_;
         float wheelPos[4][2] = {{0}};
        void InitializeWheelPositions()
        {
            wheelPos[0][0] = -params_.a; wheelPos[0][1] = params_.b;   // 左前
            wheelPos[1][0] = params_.a;  wheelPos[1][1] = params_.b;   // 右前
            wheelPos[2][0] = params_.a;  wheelPos[2][1] = -params_.b;  // 右后
            wheelPos[3][0] = -params_.a; wheelPos[3][1] = -params_.b;  // 左后
        }
        void LimitSpeed(float& speed, float maxSpeed) 
        {
            speed = Tools_t::clamp(speed, -maxSpeed, maxSpeed);
        }
        float NormalizeAngle(float angle, float tar_angle)
        {
            while (angle > tar_angle) 
                angle -= tar_angle;
            while (angle < -tar_angle) 
                angle += tar_angle;  
            return angle;
        }
            // 轮组方位角
        const float Wheel_Azimuth[4] = {M_PI / 4.0f,
                                    3.0f * M_PI / 4.0f,
                                    5.0f * M_PI / 4.0f,
                                    7.0f * M_PI / 4.0f,};

    };
}

class SteeringWheel : public Alg::Chassis::ChassisBase
{
    private:
        // 舵向电机（LK电机）和轮向电机（DJI电机）的引用
        BSP::Motor::LK::LK4005<4>& steer_motors;
        BSP::Motor::Dji::GM3508<4>& wheel_motors;
        // 底盘物理参数
        const float Wheel_Radius = 0.05f;                  // 轮半径（米）
        const float Wheel_To_Core_Distance[4] = {0.3f, 0.3f, 0.3f, 0.3f}; // 轮到中心距离
        const float Wheel_Azimuth[4] = {0, M_PI/2, M_PI, 3*M_PI/2}; // 轮安装方位角（弧度）
        float current_steer_angles[4] = {0};
        // 控制参数
        const float VW_THRESHOLD = 2.0f;
        // 防单轮超速系数
        const float WHEEL_SPEED_LIMIT_FACTOR = 0.5f;
    public:
        // 构造函数传入电机实例
        SteeringWheel(BSP::Motor::LK::LK4005<4>& steer, BSP::Motor::Dji::GM3508<4>& wheel)
            : steer_motors(steer), wheel_motors(wheel) 
            {
                // 设置底盘尺寸（根据Wheel_To_Core_Distance计算）
                float a = Wheel_To_Core_Distance[0] * cosf(M_PI/4.0f);  // X方向半长
                float b = Wheel_To_Core_Distance[0] * sinf(M_PI/4.0f);  // Y方向半宽
                SetChassisDimensions(a, b);
            }
        void UpDate(float vx, float vy, float vw, float maxSpeed)
        {
            this->params_.Vx = vx;
            this->params_.Vy = vy;
            this->params_.Vw = vw;
            for(int i = 0; i < 4; i++) 
            {
                current_steer_angles[i] = steer_motors.getAngleRad(i+1);
            }
            Kinematics_Inverse_Resolution(vx, vy, vw, maxSpeed);
        }
        void _Steer_Motor_Kinematics_Nearest_Transposition();
        void Kinematics_Inverse_Resolution(float vx, float vy, float vw, float maxSpeed);
        void Kinematics_Forward_Resolution();
        void DynamicsInverseCalculation();    

};

