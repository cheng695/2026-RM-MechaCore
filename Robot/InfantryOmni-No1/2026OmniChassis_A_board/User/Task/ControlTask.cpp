#include "ControlTask.hpp"

/**
 * @brief 初始化
 */
extern bool alphabet[28];

/* 宏定义 --------------------------------------------------------------------------------------------------*/
#define PI_ 3.1415926535897932384626433832795
#define STANDARD 0.22f  // 正方向（弧度）

/* 有限状态机 -----------------------------------------------------------------------------------------------*/
Chassis_FSM chassis_fsm;    // 底盘有限状态机

/* 底盘解算 -------------------------------------------------------------------------------------------------*/
float wheel_azimuth[4] = {-PI_/4, PI_/4, -5*PI_/4, 5*PI_/4};                // 轮安装方位角
float wheel_direction[4] = {5*PI_/4, -PI_/4, PI_/4, -5*PI_/4}; // 轮子的位置方向角
Alg::CalculationBase::Omni_IK omni_ik(0.24f, 0.07f, wheel_azimuth, wheel_direction);        // 运动学逆解算
Alg::CalculationBase::Omni_FK omni_fk(0.24f, 0.07f, 4.0f, wheel_azimuth, wheel_direction);  // 运动学正解算

/* 控制器 --------------------------------------------------------------------------------------------------*/
// 用于运动学逆解，轮向作为补偿输出
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 轮向速度pid 1号轮
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 轮向速度pid 2号轮
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 轮向速度pid 3号轮
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 150.0f)      // 轮向速度pid 4号轮
};  

ALG::PID::PID follow_pid(8.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);  // 速度环pid 用于底盘跟随

ALG::PID::PID energy_pid[2] = {
    ALG::PID::PID(15.0f, 0.0f, 10.0f, 16384.0f, 0.0f, 0.0f),     // 富足环
    ALG::PID::PID(4.0f, 0.0f, 10.0f, 16384.0f, 0.0f, 0.0f)      // 贫困环
};

/* 功率控制 -------------------------------------------------------------------------------------------------*/
ALG::PowerControl::PowerControl<4> power3508;   // 3508功率控制算法
ALG::PowerControl::EnergyRing energy_ring(1288.5f, 250.0f, 48.0f);  // 能量环
ALG::PowerControl::PowerControlStrategy power_strategy(1288.5f); // 上限功率和剩余能量逻辑处理
float coefficients3508[6] = { 2.144951, -0.002828, 0.000025, 0.016525,  0.115369, 0.000015 };   // 3508

/* 期望值与输出 ----------------------------------------------------------------------------------------------*/
Alg::Utility::SlopePlanning omni_target[3] = {
    Alg::Utility::SlopePlanning(0.015f, 0.015f),    // X轴斜坡规划
    Alg::Utility::SlopePlanning(0.015f, 0.015f),    // Y轴斜坡规划
    Alg::Utility::SlopePlanning(0.05f, 0.05f)       // Z轴旋转斜坡规划
};

ControlTask chassis_target;     // 底盘目标
Output_chassis chassis_output;  // 底盘输出

/**
 * @brief 底盘控制逻辑
 */
/* 底盘部分 ---------------------------------------------------------------------------------------------------*/

/**
 * @brief 检查所有关键设备是否在线
 * 
 * @return true 设备全部在线
 * @return false 存在离线设备
 */
bool check_online()
{
    bool isconnected = true;
    for(int i = 0; i < 4; i++)
    {
        if(!Motor3508.isConnected(i+1, i+1))
        {
            isconnected = false;
        }
    }

    if(/*!Cboard.isConnected() ||*/ !DT7.isConnected())
    {
        isconnected = false;
    }

    if(RM_RefereeSystem::RM_RefereeSystemDir())
    {
        // isconnected = false;
    }
    
    if(!isconnected)
    {
        return false;
    }

 
    
    return true;
}

/**
 * @brief 状态机初始化
 */
void fsm_init()
{
    chassis_fsm.Init();
}

/**
 * @param theta 云台角度（单位 弧度）
 * @param vx 笛卡尔坐标系的X方向输入（左右）
 * @param vy 笛卡尔坐标系的Y方向输入（前后）
 * @param phi 角度偏差补偿（单位 弧度）
 * @param out_x [输出] 计算旋转矩阵后的X方向速度（前后）
 * @param out_y [输出] 计算旋转矩阵后的Y方向速度（左右）
 */
void CalculateTranslation_xy(float theta, float vx, float vy, float phi, float *out_vx, float *out_vy, float psi)
{
    // 获取云台相对于底盘的 Yaw 并且做到角度补偿
    theta += phi;

    // 过零处理 (归一化到 -PI ~ PI)
    theta = fmod(theta, 2 * PI_);
    if (theta > PI_) theta -= 2 * PI_;
    else if (theta < -PI_) theta += 2 * PI_;

    float s = sinf(theta + psi);
    float c = cosf(theta + psi);
    // 控制量输入 (云台系)
    float raw_vx = 4.5f * vy; // raw_vx是机器人坐标系（东北天） vy是笛卡尔坐标系（x水平）
    float raw_vy = 4.5f * vx; // raw_vy是机器人坐标系（东北天） vx是笛卡尔坐标系（x水平）
    // 旋转到底盘系
    *out_vx = raw_vx * c + raw_vy * s; 
    *out_vy = raw_vx * -s + raw_vy * c;
}


/**
 * @brief 底盘跟随w规划
 */
void CalculateFollow()
{
    float follow_error = STANDARD - Cboard.GetYawAngle();
    // 归一化处理
    while (follow_error > 1.5707963267f) follow_error -= 2 * 1.5707963267f;
    while (follow_error < -1.5707963267f) follow_error += 2 * 1.5707963267f;

    // 死区处理 (防止小误差震荡)
    if(fabs(follow_error) < 0.1f) follow_error = 0.0f;
    
    follow_pid.UpDate(0.0f, follow_error);
}

/**
 * @brief 设置底盘的目标值
 * 
 * 根据当前工作模式设置相应的目标值
 */
void SetTarget()
{
    // 设置死区
    DT7.SetDeadzone(20.0f);
    // 底盘
    switch(chassis_fsm.Get_Now_State()) 
    {
        case STOP:  // 停止模式
            chassis_target.target_translation_x = 0.0f;
            chassis_target.target_translation_y = 0.0f;
            chassis_target.target_rotation = 0.0f;
            break;
        case FOLLOW:  // 底盘跟随
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)  // 键鼠模式的底盘跟随
            {
                float vx, vy;
                float psi = 0.0f; // 不小陀螺的时候不补偿
                
                float vx_Handle, vy_Handle, vw_Handle;
                vx_Handle = alphabet[22] - alphabet[18];
                vy_Handle = alphabet[3] - alphabet[0];
                vw_Handle = alphabet[23];
                
                // 小陀螺时进行相位补偿
                if (vw_Handle != 0.0f) 
                {
                    //psi = -0.08f*13.00f*vw_Handle;
                    psi = 0.0f;
                }
                
                CalculateTranslation_xy(0.0f/*Cboard.GetYawAngle()*/, vy_Handle, vx_Handle, 0.0f, &vx, &vy, psi);  // 计算旋转矩阵
                omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                chassis_target.target_translation_x = omni_target[0].GetOut();    // X方向期望（前后）
                chassis_target.target_translation_y = omni_target[1].GetOut();    // Y方向期望（左右）
                if(alphabet[23])   // 小陀螺
                {
                    omni_target[2].TIM_Calculate_PeriodElapsedCallback(13.00f * vw_Handle, omni_fk.GetChassisVw()); 
                    chassis_target.target_rotation = omni_target[2].GetOut(); // 小陀螺的期望
                }
                else    // 底盘跟随
                {
                    CalculateFollow();  // 计算底盘跟随的规划
                    chassis_target.target_rotation = follow_pid.getOutput();    // Z轴底盘跟随旋转期望
                }                
            }   
            else    // 遥控器模式的底盘跟随
            {
                float vx, vy;
                float psi = 0.0f;   // 不小陀螺的时候不补偿
                // 只在陀螺模式下进行相位补偿
                if (Cboard.GetScroll() == false && fabs(DT7.get_scroll_()) > 0.05f) 
                {
                    // psi = -0.08f * 13.00f * DT7.get_scroll_();
                    psi = 0.0f;
                }

                CalculateTranslation_xy(0.0f/*Cboard.GetYawAngle()*/, DT7.get_left_x(), DT7.get_left_y(), 0.0f, &vx, &vy, psi);  // 计算旋转矩阵
                omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                chassis_target.target_translation_x = omni_target[0].GetOut();    // X方向期望（前后）
                chassis_target.target_translation_y = omni_target[1].GetOut();    // Y方向期望（左右）
                if(Cboard.GetScroll() == true)  // 不能小陀螺
                {
                    CalculateFollow();  // 计算底盘跟随的规划
                    chassis_target.target_rotation = follow_pid.getOutput();    // Z轴底盘跟随旋转期望
                }
                else    // 可以小陀螺
                {
                    if(fabs( DT7.get_scroll_() ) > 0.05f)   // 小陀螺
                    {
                        omni_target[2].TIM_Calculate_PeriodElapsedCallback(13.00f * DT7.get_scroll_(), omni_fk.GetChassisVw()); 
                        chassis_target.target_rotation = omni_target[2].GetOut(); // 小陀螺的期望
                    }
                    else    // 底盘跟随
                    {
                        CalculateFollow();  // 计算底盘跟随的规划
                        chassis_target.target_rotation = follow_pid.getOutput();    // Z轴底盘跟随旋转期望
                    }
                }
            }
            break;
        case NOTFOLLOW:
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)  // 键鼠模式
            {
                float vx, vy;
                float psi = 0.0f;   // 不小陀螺的时候不补偿

                float vx_Handle, vy_Handle, vw_Handle;
                vx_Handle = alphabet[22] - alphabet[18];
                vy_Handle = alphabet[3] - alphabet[0];
                vw_Handle = alphabet[23];
                
                // 小陀螺时进行相位补偿
                if (vw_Handle != 0.0f) 
                {
                    //psi = -0.08f*13.00f*vw_Handle;
                    psi = 0.0f;
                }
                
                CalculateTranslation_xy(0.0f/*Cboard.GetYawAngle()*/, vy_Handle, vx_Handle, 0.0f, &vx, &vy, psi);  // 计算旋转矩阵
                omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                chassis_target.target_translation_x = omni_target[0].GetOut();    // X方向期望（前后）
                chassis_target.target_translation_y = omni_target[1].GetOut();    // Y方向期望（左右）
                omni_target[2].TIM_Calculate_PeriodElapsedCallback(13.00f * vw_Handle, omni_fk.GetChassisVw());
                chassis_target.target_rotation = omni_target[2].GetOut(); // 小陀螺的期望
            }   
            else    // 遥控器模式
            {
                float vx, vy;
                float psi = 0.0f;   // 不小陀螺的时候不补偿
                // 只在陀螺模式下进行相位补偿
                if (Cboard.GetScroll() == false && fabs(DT7.get_scroll_()) > 0.05f)
                { 
                    // psi = -0.08f * 13.00f * DT7.get_scroll_();
                    psi = 0.0f;
                }
                CalculateTranslation_xy(0.0f/*Cboard.GetYawAngle()*/, DT7.get_left_x(), DT7.get_left_y(), 0.0f, &vx, &vy, psi);  // 计算旋转矩阵
                omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                chassis_target.target_translation_x = omni_target[0].GetOut();    // X方向期望（前后）
                chassis_target.target_translation_y = omni_target[1].GetOut();    // Y方向期望（左右）
                if(Cboard.GetScroll() == true)  // 不能小陀螺
                {
                    chassis_target.target_rotation = 0.0f;  // Z轴不旋转
                }
                else    // 可以小陀螺
                {
                    omni_target[2].TIM_Calculate_PeriodElapsedCallback(13.00f * DT7.get_scroll_(), omni_fk.GetChassisVw());
                    chassis_target.target_rotation = omni_target[2].GetOut(); // 小陀螺的期望
                }
            }
            break;
        default:
            chassis_target.target_translation_x = 0.0f;
            chassis_target.target_translation_y = 0.0f;
            chassis_target.target_rotation = 0.0f;
            break;
    }
}

/**
 * @brief 底盘停止模式控制函数
 * 
 * 重置控制器参数
 */
void chassis_stop()
{
    for(int i = 0; i < 4; i++)
    {
        // 重置控制器
        wheel_pid[i].reset();
        // 输出置零
        chassis_output.out_wheel[i] = 0.0f;
    }
}

/**
 * @brief 底盘不跟随控制函数
 * 
 * 实现底盘不跟随控制
 */
void chassis_notfollow()
{
    // 正运动学解算当前底盘整体速度；逆运动学解算电机转速
    omni_fk.OmniForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    omni_ik.OmniInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 1.0f, 1.0f);
    
    for(int i = 0; i < 4; i++)
    {
        // 逆运动学相关 通过PID算轮向补偿
        wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRads(i+1));
        chassis_output.out_wheel[i] = wheel_pid[i].getOutput();
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
    
    // 功率控制
    bool isSupercapOnline = supercap.isConnected(); // 电容连接状态
    bool isRefereeOnline = !RM_RefereeSystem::RM_RefereeSystemDir(); // 裁判系统连接状态
    
    supercap.setSupercapOnline(isSupercapOnline);
    supercap.setRefereeOnline(isRefereeOnline);

    // 更新功率策略
    power_strategy.Update(isSupercapOnline, isRefereeOnline, 
                          (float)ext_power_heat_data_0x0201.chassis_power_limit, 
                          ext_power_heat_data_0x0202.chassis_power_buffer, 
                          supercap.GetCurrentEnergy(), alphabet);

    float input_limit = power_strategy.GetInputLimit(); // 基础上限功率
    float input_energy = power_strategy.GetInputEnergy();   // 剩余能量

    energy_pid[0].UpDate(sqrtf(energy_ring.GetAbundanceLine()), sqrtf(input_energy));   // 富足环
    energy_pid[1].UpDate(sqrtf(energy_ring.GetPovertyLine()), sqrtf(input_energy));     // 贫困环
    // 能量环逻辑
    energy_ring.energyring(energy_pid[0].getOutput(), energy_pid[1].getOutput(), input_limit, input_energy, alphabet[26], alphabet[27]);
    float PowerMax = energy_ring.GetPowerMax(); // 最终上限功率

    supercap.setInstruction(0); // 开启超电
    supercap.setRatedPower(input_limit);    // 给超电设置裁判系统上限功率
    supercap.setBufferEnergy(ext_power_heat_data_0x0202.chassis_power_buffer);  // 给超电设置剩余缓冲能量

    float I3508[4], I_other[4], V3508[4];   // 必要参数
    for(int i = 0; i < 4; i++)
    {
        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;    // 3508电机电流（解算欲输出电流）
        I_other[i] = 0.0f;                                          // 3508电机电流（非解算欲输出电流）
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f);  // 3508电机速度（当前速度）
    }
    float pmax3508 = PowerMax; // 3508电机吃满
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508); // 3508电机功率控制（衰减电流法）

    // 底盘输出
    for(int i = 0; i < 4; i++)
    {
        //chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;
    }
}

/**
 * @brief 底盘跟随控制函数
 * 
 * 实现底盘跟随控制
 */
void chassis_follow()
{   
    // 正运动学解算当前底盘整体速度；逆运动学解算电机转速
    omni_fk.OmniForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    omni_ik.OmniInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 1.0f, 1.0f);
    
    for(int i = 0; i < 4; i++)
    {
        // 逆运动学相关 通过PID算舵向输出和轮向补偿
        wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRads(i+1));
        chassis_output.out_wheel[i] = wheel_pid[i].getOutput();
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
    
    // 功率控制
    bool isSupercapOnline = supercap.isConnected();
    bool isRefereeOnline = !RM_RefereeSystem::RM_RefereeSystemDir();

    supercap.setSupercapOnline(isSupercapOnline);
    supercap.setRefereeOnline(isRefereeOnline);

    // 更新功率策略
    power_strategy.Update(isSupercapOnline, isRefereeOnline, 
                          (float)ext_power_heat_data_0x0201.chassis_power_limit, 
                          ext_power_heat_data_0x0202.chassis_power_buffer, 
                          supercap.GetCurrentEnergy(), alphabet);

    float input_limit = power_strategy.GetInputLimit(); // 基础上限功率
    float input_energy = power_strategy.GetInputEnergy();   // 剩余能量

    energy_pid[0].UpDate(sqrtf(energy_ring.GetAbundanceLine()), sqrtf(input_energy));   // 富足环
    energy_pid[1].UpDate(sqrtf(energy_ring.GetPovertyLine()), sqrtf(input_energy));     // 贫困环
    // 能量环逻辑
    energy_ring.energyring(energy_pid[0].getOutput(), energy_pid[1].getOutput(), input_limit, input_energy, alphabet[26], alphabet[27]);
    float PowerMax = energy_ring.GetPowerMax(); // 最终上限功率
    
    supercap.setInstruction(0); // 开启超电
    supercap.setRatedPower(input_limit);    // 给超电设置裁判系统上限功率
    supercap.setBufferEnergy(ext_power_heat_data_0x0202.chassis_power_buffer);  // 给超电设置剩余缓冲能量

    float I3508[4], I_other[4], V3508[4];   // 必要参数
    for(int i = 0; i < 4; i++)
    {
        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;    // 3508电机电流（解算欲输出电流）
        I_other[i] = 0.0f;                                          // 3508电机电流（非解算欲输出电流）
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f); // 3508电机速度（当前速度）
    }
    float pmax3508 = PowerMax; // 3508电机吃满
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508); // 3508电机功率控制（衰减电流法）

    // 底盘输出
    for(int i = 0; i < 4; i++)
    {
        //chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;
    }
}

/**
 * @brief 底盘主控制循环
 * 
 * @param left_sw 遥控器左开关状态
 * @param right_sw 遥控器右开关状态
 * @param is_online 设备在线状态
 */
void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online, bool *alphabet) 
{   
    chassis_fsm.StateUpdate(left_sw, right_sw, is_online, alphabet);
    SetTarget();

    switch(chassis_fsm.Get_Now_State()) 
    {
        case STOP:
            chassis_stop();
            break;
        case FOLLOW:
            chassis_follow();
            break;
        case NOTFOLLOW:
            chassis_notfollow();
            break;
        default:
            chassis_stop();
            break;
    }
}

/* 控制任务部分 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 控制任务主函数
 * 
 * @param argument 任务参数
 */
extern "C"{
void Control(void const * argument)
{
    // 初始化蜂鸣器管理器
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    fsm_init();
    for(;;)
    {
        // 更新蜂鸣器管理器，处理队列中的响铃请求
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online(), alphabet);

        osDelay(1);
    } 
}
}