#include "PowerLimitTask.hpp"
#include "cmsis_os2.h"
#include "math.h"
#include <typeinfo>

// 全局对象定义
Tools_t Tools_p;
BSP::Motor::Dji::GM6020<4> Motor6020(0x204, {1,2,3,4}, 0x1FE);
BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1,2,3,4}, 0x200);
PowerControlTask GlobalPowerControl;

// 扭矩常数定义（需要根据实际电机参数调整）
const float toque_const_3508 = 0.3f;  // GM3508扭矩常数 (Nm/A)
const float toque_const_6020 = 0.7f;  // GM6020扭矩常数 (Nm/A)

// ==================== 辅助函数实现 ====================

/**
 * @brief 获取电机数据的辅助函数
 */
MotorData GetMotorData(BSP::Motor::Dji::Dji_Motor& motor, uint8_t motor_id) {
    MotorData data;
    
    data.speed = motor.getVelocityRads(motor_id);
    data.torque = motor.getTorque(motor_id);
    
    // 根据电机类型设置参数
    if (typeid(motor) == typeid(BSP::Motor::Dji::GM3508<4>)) {
        data.torque_const = toque_const_3508;
        data.max_output = 16384.0f; // 3508最大输出
    } else if (typeid(motor) == typeid(BSP::Motor::Dji::GM6020<4>)) {
        data.torque_const = toque_const_6020;
        data.max_output = 30000.0f; // 6020最大输出
    } else {
        data.torque_const = toque_const_3508;
        data.max_output = 16384.0f;
    }
    
    return data;
}

// ==================== PowerDataManager 实现 ====================

PowerDataManager::PowerDataManager()
{
    // 初始化默认参数
    target_base_power = 20.0f;   // 20W基础功率
    target_full_power = 80.0f;   // 80W全功率
    base_kp = 2.0f;
    full_kp = 1.5f;
    base_max_power = 40.0f;
    full_max_power = 100.0f;
    
    estimated_power = 0.0f;
    current_power = 0.0f;
    power_limit = 60.0f;
    
    // 清零采样数据
    memset(wheel_samples, 0, sizeof(wheel_samples));
    memset(steer_samples, 0, sizeof(steer_samples));
}

void PowerDataManager::LoadOfflineParams(const OfflineParams& wheel, const OfflineParams& steer)
{
    wheel_params = wheel;
    steer_params = steer;
}

void PowerDataManager::UpdatePowerEstimation()
{
    estimated_power = 0.0f;
    effective_power_steer = 0.0f;
    effective_power_wheel = 0.0f;
    
    // 计算舵向电机功率
    CalculateMotorPower(Motor6020, steer_params, steer_samples, effective_power_steer);
    
    // 计算轮向电机功率  
    CalculateMotorPower(Motor3508, wheel_params, wheel_samples, effective_power_wheel);
    
    // 总估算功率 = 舵向功率 + 轮向功率
    estimated_power = effective_power_steer + effective_power_wheel;
    
    // 更新当前实际功率（从电源管理模块读取）
    current_power = BSP::Power::pm01.cin_power;
}

void PowerDataManager::CalculateMotorPower(BSP::Motor::Dji::Dji_Motor& motor, 
                                         const OfflineParams& params,
                                         float samples[2][4],
                                         float& effective_power)
{
    effective_power = 0.0f;
    float total_sample0 = 0.0f; // Σ|ω|
    float total_sample1 = 0.0f; // ΣT²
    
    for (int i = 0; i < 4; i++) {
        uint8_t motor_id = i + 1;
        
        // 使用电机库接口获取数据
        float torque = motor.getTorque(motor_id);
        float speed = motor.getVelocityRads(motor_id);
        
        // 有效机械功率: T × ω
        effective_power += torque * speed;
        
        // 采样数据用于损耗计算
        samples[0][i] = fabs(speed);          // |ω|
        samples[1][i] = torque * torque;      // T²
        
        total_sample0 += samples[0][i];
        total_sample1 += samples[1][i];
    }
    
    // 添加损耗分量: k1*Σ|ω| + k2*ΣT² + k3
    effective_power += params.k1 * total_sample0 + params.k2 * total_sample1 + params.k3;
}

void PowerDataManager::EnergyLoop()
{
    // 计算基础功率和全功率的电压误差（平方根线性化）
    float base_err = sqrt(target_base_power) - sqrt(BSP::Power::pm01.cout_voltage);
    float full_err = sqrt(target_full_power) - sqrt(BSP::Power::pm01.cout_voltage);

    // 计算基础最大功率和全最大功率（考虑误差补偿）
    base_max_power = fmax(power_limit - base_err * base_kp, 15.0f);
    full_max_power = fmax(power_limit - full_err * full_kp, 15.0f);
}

float PowerDataManager::GetAvailableMaxPower() const
{
    // 电压高于24V×0.9时使用全功率模式，否则使用基础功率
    if (BSP::Power::pm01.cout_voltage > 24 * 0.9) {
        return full_max_power;
    } else {
        return base_max_power;
    }
}

// ==================== PowerLimitController 实现 ====================

PowerLimitController::PowerLimitController()
{
    memset(cmd_max_torque, 0, sizeof(cmd_max_torque));
    memset(p_max_power, 0, sizeof(p_max_power));
}

void PowerLimitController::CalculateMaxTorque(float* final_output, 
                                            BSP::Motor::Dji::Dji_Motor& motor,
                                            PID* pid_controller,
                                            const PowerDataManager::OfflineParams& params,
                                            float max_power)
{
    for (int i = 0; i < 4; i++) {
        uint8_t motor_id = i + 1;
        
        // 获取电机数据
        MotorData motor_data = GetMotorData(motor, motor_id);
        
        // 求解二次方程得到限制后的扭矩
        cmd_max_torque[i] = SolveQuadraticForTorque(motor_data.speed, params.k1, params.k2, 
                                                   params.k3, p_max_power[i], motor_data.torque_const);
        
        // 钳位到安全范围
        cmd_max_torque[i] = Tools_p.clamp(cmd_max_torque[i], -motor_data.max_output, motor_data.max_output);
        
        // 更新最终输出
        final_output[i] = cmd_max_torque[i];
    }
}

void PowerLimitController::ScalePowerAllocation(PID* pid_controller, BSP::Motor::Dji::Dji_Motor& motor, float max_power)
{
    // 计算所有电机PID误差的绝对值总和
    float sum_err = 0.0f;
    for (int i = 0; i < 4; i++) {
        sum_err += fabsf(pid_controller[i].GetErr());
    }
    
    if (sum_err < 1e-5f) {
        // 误差过小时平均分配
        for (int i = 0; i < 4; i++) {
            p_max_power[i] = max_power / 4.0f;
        }
    } else {
        // 根据误差比例分配功率
        for (int i = 0; i < 4; i++) {
            p_max_power[i] = max_power * (fabsf(pid_controller[i].GetErr()) / sum_err);
        }
    }
}

float PowerLimitController::SolveQuadraticForTorque(float omega, float k1, float k2, float k3, 
                                                  float max_power, float torque_const)
{
    // 构建二次方程: A×T² + B×T + C = 0
    float A = k2;
    float B = omega;
    float C = k1 * fabs(omega) + k3 / 4.0f - max_power;
    
    // 计算判别式
    float delta = B * B - 4.0f * A * C;
    
    if (delta <= 0) {
        // 无实数解或重根，使用简化解
        return -B / (2.0f * A) / torque_const;
    }
    
    // 根据扭矩方向选择合适的根
    float root1 = (-B + sqrtf(delta)) / (2.0f * A);
    float root2 = (-B - sqrtf(delta)) / (2.0f * A);
    
    // 选择正扭矩对应的解（假设电机正常工作时扭矩与速度同向）
    return (omega >= 0) ? (root1 / torque_const) : (root2 / torque_const);
}

// ==================== PowerControlTask 实现 ====================

PowerControlTask::PowerControlTask() 
    : total_power(0.0f), steer_power(0.0f), wheel_power(0.0f),
      effective_power_steer(0.0f), effective_power_wheel(0.0f)
{
}

bool PowerControlTask::Initialize()
{
    // 设置默认离线拟合参数（应从配置文件加载）
    PowerDataManager::OfflineParams wheel_params, steer_params;
    
    // 3508轮向电机典型参数（需要根据实际拟合调整）
    wheel_params.k1 = 0.0012f;   // 铜损系数
    wheel_params.k2 = 0.0008f;   // 机械功率系数  
    wheel_params.k3 = 0.0003f;   // 铁损系数
    wheel_params.k4 = 2.5f;      // 固定损耗
    
    // 6020舵向电机典型参数（需要根据实际拟合调整）
    steer_params.k1 = 0.0015f;
    steer_params.k2 = 0.0006f;  
    steer_params.k3 = 0.0004f;
    steer_params.k4 = 3.0f;
    
    power_data.LoadOfflineParams(wheel_params, steer_params);
    
    // 设置功率分配比例
    limit_controller.allocation.steer_ratio = 0.4f;
    limit_controller.allocation.wheel_ratio = 0.6f;
    
    return true;
}

void PowerControlTask::Execute()
{
    //  更新能量环状态
    power_data.EnergyLoop();
    
    // 更新功率估算
    power_data.UpdatePowerEstimation();
    
    //  获取裁判系统功率限制
    float chassis_limit = ext_power_heat_data_0x0201.chassis_power_limit;
    SetPowerLimit(chassis_limit);
    
    //  更新超级电容控制
    BSP::SuperCap::cap.SetSendValue(chassis_limit - 1.0f); // 留1W余量
    BSP::SuperCap::cap.sendCAN(&hcan2, CAN_TX_MAILBOX0);
    
    //  应用功率限制到电机输出
    UpdateMotorOutputs();
}

void PowerControlTask::SetPowerLimit(float limit)
{
    power_data.power_limit = limit;
}

void PowerControlTask::UpdateMotorOutputs()
{
    float available_power = power_data.GetAvailableMaxPower();
    
    // 如果估算功率超过可用功率，应用限制
    if (power_data.estimated_power > available_power) {
        // 舵向电机功率限制
        float steer_power_limit = available_power * limit_controller.allocation.steer_ratio;
        limit_controller.ScalePowerAllocation(pid_vel_String, Motor6020, steer_power_limit);
        limit_controller.CalculateMaxTorque(Chassis_Data.final_6020_Out, Motor6020, 
                                          pid_vel_String, power_data.steer_params, steer_power_limit);
        
        // 轮向电机功率限制  
        float wheel_power_limit = available_power * limit_controller.allocation.wheel_ratio;
        limit_controller.ScalePowerAllocation(pid_vel_Wheel, Motor3508, wheel_power_limit);
        limit_controller.CalculateMaxTorque(Chassis_Data.final_3508_Out, Motor3508,
                                          pid_vel_Wheel, power_data.wheel_params, wheel_power_limit);
    }
    
    // 更新实时功率状态
    steer_power = power_data.effective_power_steer;
    wheel_power = power_data.effective_power_wheel;  
    total_power = power_data.estimated_power;
}

/**
 * @brief 功率控制任务入口函数
 */
void PowerTaskEntry(void *argument)
{
    osDelay(500); // 启动延迟
    
    GlobalPowerControl.Initialize();
    
    for (;;) {
        GlobalPowerControl.Execute();
        osDelay(1); // 1ms控制周期
    }
}

// ==================== 电机控制函数（兼容原有代码） ====================

/**
 * @brief 舵向电机6020双环控制
 */
void SteerMotor_6020_DualLoop()
{
    using namespace BSP::Motor::Dji;
    
    for (int i = 0; i < 4; i++)
    {
        uint8_t motor_id = i + 1; // 电机ID从1开始
        
        // 获取当前电机角度和速度
        float current_angle = Motor6020.getAngleDeg(motor_id);
        float current_omega = Motor6020.getVelocityRads(motor_id);
        
        // 角度环PID计算
        float angle_output = pid_angle_String[i].GetPidPos(Kpid_6020_angle, 
                                                          Chassis_Data.tar_angle[i],  // 目标角度
                                                          current_angle,              // 当前角度
                                                          30000.0f);                  // 输出限幅
        
        // 角度环输出作为速度环的目标
        float target_omega = angle_output;
        
        // 速度环PID计算
        float omega_output = pid_vel_String[i].GetPidPos(Kpid_6020_vel, 
                                                        target_omega, 
                                                        current_omega, 
                                                        30000.0f);                    // 输出限幅
        
        // 6020双环速度环的最终输出接口
        Chassis_Data.final_6020_Out[i] = omega_output;
    }
}

/**
 * @brief 轮向电机3508速度环控制
 */
void WheelMotor_3508_SpeedLoop()
{
    using namespace BSP::Motor::Dji;
    
    for (int i = 0; i < 4; i++)
    {
        uint8_t motor_id = i + 1; // 电机ID从1开始
        
        // 获取当前电机速度
        float current_omega = Motor3508.getVelocityRads(motor_id);
        
        // 速度环PID计算
        float wheel_output = pid_vel_Wheel[i].GetPidPos(Kpid_3508_vel, 
                                                       Chassis_Data.tar_speed[i],  // 目标速度
                                                       current_omega, 
                                                       16384.0f);                 // 输出限幅
        
        // 3508速度环输出
        Chassis_Data.final_3508_Out[i] = wheel_output;
    }
}

/**
 * @brief 主功率控制函数（兼容原有接口）
 */
void Power(void)
{
    // 首先执行电机PID控制计算
    SteerMotor_6020_DualLoop();  // 6020双环控制
    WheelMotor_3508_SpeedLoop(); // 3508速度环控制

    // 执行全局功率控制
    GlobalPowerControl.Execute();
}