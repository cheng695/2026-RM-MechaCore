/**
 * @file PowerTask.hpp
 * @brief 离线拟合功率控制任务头文件
 * @details 基于物理模型的功率估算和限制控制，使用离线拟合参数
 */

#pragma once
#include "Variable.hpp"
#include "RM_RefereeSystem.h"
#include "drv_math.h"
#include "Tools.hpp"
// #include "../BSP/Power/PM01.hpp"
// #include "../BSP/SuperCap/SuperCap.hpp"
#define My_PI 3.14152653529799323                    // 圆周率π的近似值

// 电机参数定义
#define toque_const_3508 0.00036621                  // 3508电机扭矩常数 (Nm/A)
#define rpm_to_rads_3508 0.0029088820f               // 3508电机RPM转rad/s换算系数

#define toque_const_6020 0.000128173828f             // 6020电机扭矩常数 (Nm/A)  
#define rpm_to_rads_6020 0.104719555f                // 6020电机RPM转rad/s换算系数

#define pMAX 150.0f                                  // 最大功率限制值 (W)


// 声明全局对象
extern Tools_t Tools_p;
extern BSP::Motor::Dji::GM6020<4> Motor6020;
extern BSP::Motor::Dji::GM3508<4> Motor3508;

/**
 * @brief 功率数据管理类
 * @detail 管理离线拟合的功率模型参数和实时功率估算
 */
class PowerDataManager
{
public:
    // 离线拟合参数 - 基于物理模型
    struct OfflineParams {
        float k1; // 铜损系数 (I²R损耗)
        float k2; // 机械功率系数 (T×W)  
        float k3; // 铁损系数 (W²铁损)
        float k4; // 固定损耗 (常数项)
        
        OfflineParams() : k1(1e-5f), k2(1e-5f), k3(1e-5f), k4(0.0f) {}
    };

    OfflineParams wheel_params;  // 轮向电机3508参数
    OfflineParams steer_params;  // 舵向电机6020参数
    
    // 功率状态
    float estimated_power;       // 总估算功率
    float current_power;         // 当前实际功率
    float power_limit;           // 功率限制值
    
    // 能量环参数
    float target_base_power;     // 基础功率目标
    float target_full_power;     // 全功率目标  
    float base_kp, full_kp;      // 能量环比例系数
    float base_max_power;        // 基础最大功率
    float full_max_power;        // 全最大功率

    PowerDataManager();
    
    /**
     * @brief 加载离线拟合参数
     */
    void LoadOfflineParams(const OfflineParams& wheel, const OfflineParams& steer);
    
    /**
     * @brief 更新功率估算
     */
    void UpdatePowerEstimation();
    
    /**
     * @brief 能量环控制
     */
    void EnergyLoop();
    
    /**
     * @brief 获取当前可用最大功率
     */
    float GetAvailableMaxPower() const;

private:
    // 采样数据缓存
    float wheel_samples[2][4];   // 轮向电机采样数据
    float steer_samples[2][4];   // 舵向电机采样数据
    
    void CalculateMotorPower(BSP::Motor::Dji::Dji_Motor& motor, 
                           const OfflineParams& params, 
                           float samples[2][4],
                           float& effective_power);
};

/**
 * @brief 功率限制控制器
 * @detail 基于二次方程解析解的功率限制算法
 */
class PowerLimitController
{
public:
    // 功率分配配置
    struct PowerAllocation {
        float steer_ratio;       // 舵向功率分配比例
        float wheel_ratio;       // 轮向功率分配比例
        float steer_max_output;  // 舵向最大输出
        float wheel_max_output;  // 轮向最大输出
        
        PowerAllocation() : steer_ratio(0.4f), wheel_ratio(0.6f), 
                          steer_max_output(30000.0f), wheel_max_output(16384.0f) {}
    };

    PowerAllocation allocation;
    
    // 限制状态
    float cmd_max_torque[4];     // 各电机最大扭矩指令
    float p_max_power[4];        // 各电机最大分配功率

    PowerLimitController();
    
    /**
     * @brief 计算功率限制后的最大扭矩
     */
    void CalculateMaxTorque(float* final_output, 
                          BSP::Motor::Dji::Dji_Motor& motor,
                          PID* pid_controller,
                          const PowerDataManager::OfflineParams& params,
                          float max_power);
    
    /**
     * @brief 等比缩放功率分配
     */
    void ScalePowerAllocation(PID* pid_controller, float max_power);

private:
    /**
     * @brief 求解二次方程得到限制扭矩
     */
    float SolveQuadraticForTorque(float omega, float k1, float k2, float k3, 
                                float max_power, float torque_constant);
};

/**
 * @brief 主功率控制任务
 * @detail 集成功率估算和限制控制的完整解决方案
 */
class PowerControlTask
{
public:
    PowerDataManager power_data;
    PowerLimitController limit_controller;
    
    // 实时功率状态
    float total_power;           // 系统总功率
    float steer_power;           // 舵向功率
    float wheel_power;           // 轮向功率
    

        // 功率计算临时变量
    float effective_power_steer;
    float effective_power_wheel;
    
    PowerControlTask();
    
    /**
     * @brief 初始化功率控制任务
     */
    bool Initialize();
    
    /**
     * @brief 执行功率控制循环
     */
    void Execute();
    
    /**
     * @brief 设置功率限制
     */
    void SetPowerLimit(float limit);
    
    /**
     * @brief 获取当前功率状态
     */
    float GetCurrentPower() const { return total_power; }
    
    /**
     * @brief 更新电机控制输出（应用功率限制）
     */
    void UpdateMotorOutputs();

private:

    /**
     * @brief 更新所有电机采样数据
     */
    void UpdateMotorSamples();
};

// 全局功率控制实例
extern PowerControlTask GlobalPowerControl;