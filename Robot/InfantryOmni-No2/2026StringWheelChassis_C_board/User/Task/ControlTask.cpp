#include "ControlTask.hpp"
#include "FiniteStateMachine_chassis.hpp"
#include "StringWheel.hpp"

float target_4005_angle[4];
float board[3] = {0.0f, 0.0f, 0.0f};
/**
 * @brief 初始化
 */
extern bool alphabet[28];

/* 宏定义 --------------------------------------------------------------------------------------------------*/
#define PI_ 3.1415926535897932384626433832795
#define STANDARD -0.22418f  // 正方向（弧度）

/* 有限状态机 -----------------------------------------------------------------------------------------------*/
Chassis_FSM chassis_fsm;    // 底盘有限状态机

/* 底盘解算 -------------------------------------------------------------------------------------------------*/
float wheel_azimuth[4] = {3*PI_/4, 5*PI_/4, 7*PI_/4, PI_/4};                // 安装位置
float phase_offset[4] = {0.2825f, -2.4902f,-1.5221f, 3.7968f};         // 舵向初始角
Alg::CalculationBase::String_IK string_ik(0.22f, 0.09f, wheel_azimuth, phase_offset);  // 运动学逆解算
Alg::CalculationBase::String_FK string_fk(0.22f, 0.09f, wheel_azimuth, phase_offset);  // 运动学正解算
Alg::CalculationBase::String_ID string_id(0.22f, 0.09f, wheel_azimuth, phase_offset);  // 动力学逆解算
Alg::CalculationBase::String_FK string_fk_odometer_v(0.22f, 0.09f, wheel_azimuth, phase_offset);  // 运动学正解算（里程计速度）

/* 控制器 --------------------------------------------------------------------------------------------------*/
// 用于运动学逆解，舵向直接输出，轮向作为补偿输出
ALG::PID::PID stringAngle_pid[4] = {
    ALG::PID::PID(60.0f, 0.0f, 0.0f, 320.0f, 2500.0f, 10.0f),       // 舵向角速度pid 1号舵
    ALG::PID::PID(60.0f, 0.0f, 0.0f, 320.0f, 2500.0f, 10.0f),      // 舵向角速度pid 2号舵
    ALG::PID::PID(60.0f, 0.0f, 0.0f, 320.0f, 2500.0f, 10.0f),      // 舵向角速度pid 3号舵
    ALG::PID::PID(60.0f, 0.0f, 0.0f, 320.0f, 2500.0f, 20.0f)       // 舵向角速度pid 4号舵
};  
ALG::PID::PID stringVelocity_pid[4] = {
    ALG::PID::PID(1.0f, 0.0f, 0.0f, 2000.0f, 2500.0f, 100.0f),    // 舵向角速度pid 1号舵
    ALG::PID::PID(1.0f, 0.0f, 0.0f, 2000.0f, 2500.0f, 100.0f),    // 舵向角速度pid 2号舵
    ALG::PID::PID(1.0f, 0.0f, 0.0f, 2000.0f, 2500.0f, 100.0f),    // 舵向角速度pid 3号舵
    ALG::PID::PID(1.0f, 0.0f, 0.0f, 2000.0f, 2500.0f, 100.0f)     // 舵向角速度pid 4号舵
}; 
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 轮向速度pid 1号轮
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 轮向速度pid 2号轮
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 轮向速度pid 3号轮
    ALG::PID::PID(0.9f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f)      // 轮向速度pid 4号轮
};  

// 用于运动学正解后计算输出为力
ALG::PID::PID translational_pid[2] = {
    ALG::PID::PID(300.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f),   // X方向牵引力
    ALG::PID::PID(300.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f)    // Y方向牵引力
};
ALG::PID::PID rotational_pid(200.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);    // Z轴旋转力矩

ALG::PID::PID follow_pid(4.5f, 0.0f, 0.0f, 24.0f, 2500.0f, 100.0f);  // 速度环pid 用于底盘跟随

ALG::PID::PID energy_pid[2] = {
    ALG::PID::PID(15.0f, 0.0f, 10.0f, 16384.0f, 0.0f, 0.0f),     // 富足环
    ALG::PID::PID(4.0f, 0.0f, 10.0f, 16384.0f, 0.0f, 0.0f)      // 贫困环
};

/* 功率控制 -------------------------------------------------------------------------------------------------*/
ALG::PowerControl::PowerControl<4> power3508;   // 3508功率控制算法
ALG::PowerControl::PowerControl<4> power4005;   // 4005功率控制算法
ALG::PowerControl::EnergyRing energy_ring(1288.5f, 250.0f, 48.0f);  // 能量环
ALG::PowerControl::PowerControlStrategy power_strategy(1288.5f); // 上限功率和剩余能量逻辑处理
float coefficients3508[6] = { 2.144951, -0.002828, 0.000025, 0.016525,  0.115369, 0.000015 };   // 3508
float coefficients4005[6] = { 1.586024,  0.013252, 0.000229, 0.530772,  7.509297, 0.000320 };   // 4005



/* 期望值与输出 ----------------------------------------------------------------------------------------------*/
Alg::Utility::SlopePlanning string_target[2] = {
    Alg::Utility::SlopePlanning(0.01f, 0.01f),    // X轴斜坡规划
    Alg::Utility::SlopePlanning(0.01f, 0.01f)    // Y轴斜坡规划
};

ControlTask chassis_target;     // 底盘目标
Output_chassis chassis_output;  // 底盘输出
Navigation navigation;          // 导航数据

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
    //上场要注释掉，蜂鸣器使用了队列存在滞后，不注释掉的话场上复活开始的几秒轮子会疯
    // for(int i = 0; i < 4; i++)
    // {
    //     if(!Motor3508.isConnected(i+1, i+1) || !Motor4005.isConnected(i+1, i+5))
    //     {
    //         isconnected = false;
    //     }
    // }

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
    float raw_vx = 5.3f * vy; // raw_vx是机器人坐标系（东北天） vy是笛卡尔坐标系（x水平）
    float raw_vy = 5.3f * vx; // raw_vy是机器人坐标系（东北天） vx是笛卡尔坐标系（x水平）
    // 旋转到底盘系
    *out_vx = raw_vx * c + raw_vy * s; 
    *out_vy = raw_vx * -s + raw_vy * c;
}


/**
 * @brief 底盘跟随w规划
 */
void CalculateFollow()
{
    float follow_error = STANDARD - Cboard.GetFilteredYawAngle();
    // 归一化处理
    while (follow_error > 1.5707963267f) follow_error -= 2 * 1.5707963267f;
    while (follow_error < -1.5707963267f) follow_error += 2 * 1.5707963267f;

    // 死区处理 (防止小误差震荡)
    if(fabs(follow_error) < 0.1f) follow_error = 0.0f;
    
    follow_pid.UpDate(follow_error, 0.0f);
}

/**
 * @brief 限制底盘平动与旋转的合成目标速度，防止电机超速跑偏
 */
void LimitChassisVector(float &vx_norm, float &vy_norm, float &vw_norm)
{
    float MAX_LINEAR_VEL = 5.3f;   // 和 CalculateTranslation_xy 内部系数一致
    float MAX_ROTATION_VEL = 15.0f; // 小陀螺角速度期望

    float target_vx = vx_norm * MAX_LINEAR_VEL; 
    float target_vy = vy_norm * MAX_LINEAR_VEL;
    float target_vw = vw_norm * MAX_ROTATION_VEL;

    // 半径 R 约为 0.22m
    float vw_equivalent_linear = fabs(target_vw) * 0.22f;
    float v_translation_magnitude = sqrtf(target_vx * target_vx + target_vy * target_vy);
    float total_required_speed = v_translation_magnitude + vw_equivalent_linear;

    float MOTOR_PHYSICAL_LIMIT = 5.2f; // 保守极限速度，3508满转极限约为 5.2 m/s
    if (total_required_speed > MOTOR_PHYSICAL_LIMIT) 
    {
        float scale = MOTOR_PHYSICAL_LIMIT / total_required_speed;
        vx_norm *= scale;
        vy_norm *= scale;
        vw_norm *= scale;
    }
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
                
                LimitChassisVector(vx_Handle, vy_Handle, vw_Handle);

                float target_vw_raw = 0.0f;
                // 1. 小陀螺与跟随的动态斜坡策略
                if(alphabet[23])   // 小陀螺: 慢速启动和停止，极度丝滑 (0.02f)
                {
                    target_vw_raw = 15.0f * vw_Handle;
                    psi = 0.1f * target_vw_raw;
                }
                else    // 底盘跟随: 快速响应和高压制刹车 (0.15f)，杜绝走S形
                {
                    CalculateFollow();  
                    target_vw_raw = follow_pid.getOutput(); 
                }

                chassis_target.target_rotation = target_vw_raw; 

                // 3. 完美的相位补偿：跟随系统物理真实旋转速度
                
                CalculateTranslation_xy(Cboard.GetFilteredYawAngle(), vy_Handle, vx_Handle, 0.16f, &vx, &vy, psi);  // 计算旋转矩阵
                string_target[0].TIM_Calculate_PeriodElapsedCallback(vx, string_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                string_target[1].TIM_Calculate_PeriodElapsedCallback(vy, string_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                chassis_target.target_translation_x = string_target[0].GetOut();    // X方向期望（前后）
                chassis_target.target_translation_y = string_target[1].GetOut();    // Y方向期望（左右）                
            }   
            else    // 遥控器模式的底盘跟随
            {
                float vx, vy;
                float psi = 0.0f;   // 不小陀螺的时候不补偿
                
                float vx_Handle = -DT7.get_left_x();
                float vy_Handle = DT7.get_left_y();
                float vw_Handle = DT7.get_scroll_();

                if (Cboard.GetScroll() == true || fabs(vw_Handle) <= 0.05f) {
                    vw_Handle = 0.0f;
                }
                
                LimitChassisVector(vx_Handle, vy_Handle, vw_Handle);

                float target_vw_raw = 0.0f;
                // 1. 小陀螺与跟随的动态斜坡策略
                if(vw_Handle != 0.0f)   // 可以小陀螺
                {
                    target_vw_raw = 15.0f * vw_Handle;
                    psi = 0.1f * target_vw_raw;
                }
                else    // 底盘跟随
                {
                    CalculateFollow();  
                    target_vw_raw = follow_pid.getOutput(); 
                    
                    // if (target_vw_raw > 15.0f) target_vw_raw = 15.0f;
                    // if (target_vw_raw < -15.0f) target_vw_raw = -15.0f;
                }

                // 2. 核心保护：所有旋转期望统一过斜坡
                chassis_target.target_rotation = target_vw_raw; 

                // 3. 完美的相位补偿：跟随系统物理真实旋转速度
                
                CalculateTranslation_xy(Cboard.GetFilteredYawAngle(), vx_Handle, vy_Handle, 0.16f, &vx, &vy, psi);  // 计算旋转矩阵
                string_target[0].TIM_Calculate_PeriodElapsedCallback(vx, string_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                string_target[1].TIM_Calculate_PeriodElapsedCallback(vy, string_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                chassis_target.target_translation_x = string_target[0].GetOut();    // X方向期望（前后）
                chassis_target.target_translation_y = string_target[1].GetOut();    // Y方向期望（左右）
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
                
                LimitChassisVector(vx_Handle, vy_Handle, vw_Handle);
                
                float target_vw_raw = 15.0f * vw_Handle;

                chassis_target.target_rotation = target_vw_raw;

                // 使用已被斜坡平滑处理过的真实旋转速度进行绝佳的相位补偿
                psi = 0.1f * target_vw_raw;
                
                CalculateTranslation_xy(Cboard.GetFilteredYawAngle(), vy_Handle, vx_Handle, 0.16f, &vx, &vy, psi);  // 计算旋转矩阵
                string_target[0].TIM_Calculate_PeriodElapsedCallback(vx, string_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                string_target[1].TIM_Calculate_PeriodElapsedCallback(vy, string_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                chassis_target.target_translation_x = string_target[0].GetOut();    // X方向期望（前后）
                chassis_target.target_translation_y = string_target[1].GetOut();    // Y方向期望（左右）
            }   
            else    // 遥控器模式
            {
                float vx, vy;
                float psi = 0.0f;   // 不小陀螺的时候不补偿
                
                float vx_Handle = -DT7.get_left_x();
                float vy_Handle = DT7.get_left_y();
                float vw_Handle = DT7.get_scroll_();

                if (Cboard.GetScroll() == true || fabs(vw_Handle) <= 0.05f) 
                {
                    vw_Handle = 0.0f;
                }
                
                LimitChassisVector(vx_Handle, vy_Handle, vw_Handle);

                float target_vw_raw = 15.0f * vw_Handle;

                chassis_target.target_rotation = target_vw_raw;

                // 使用实时的真实旋转速度进行相位补偿
                psi = 0.1f * target_vw_raw;
                
                CalculateTranslation_xy(Cboard.GetFilteredYawAngle(), vx_Handle, vy_Handle, 0.16f, &vx, &vy, psi);  // 计算旋转矩阵
                string_target[0].TIM_Calculate_PeriodElapsedCallback(vx, string_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                string_target[1].TIM_Calculate_PeriodElapsedCallback(vy, string_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                chassis_target.target_translation_x = string_target[0].GetOut();    // X方向期望（前后）
                chassis_target.target_translation_y = string_target[1].GetOut();    // Y方向期望（左右）
            }
            break;
            case NAVIGATION:
                {
                    // 导航目标值（底盘系真实速度 m/s, rad/s）
                    float vx_target = -Cboard.GetTargetVx();
                    float vy_target = -Cboard.GetTargetVy();
                    float vw_target = Cboard.GetTargetWz();

                    // 遥控器并联输入（底盘系直接叠加，无需旋转矩阵）
                    float rc_vx = DT7.get_left_x();
                    float rc_vy = -DT7.get_left_y();
                    float rc_vw = DT7.get_scroll_();

                    if (Cboard.GetScroll() == true || fabs(rc_vw) <= 0.05f)
                    {
                        rc_vw = 0.0f;
                    }

                    // 并联叠加（遥控器直译为底盘系速度）
                    vx_target += rc_vy * 5.3f;
                    vy_target += rc_vx * 5.3f;
                    vw_target += rc_vw * 15.0f;

                    board[0] = vx_target;
                    board[1] = vy_target;
                    board[2] = vw_target;

                    // 速度限幅
                    float vw_eq = fabsf(vw_target) * 0.22f;
                    float v_xy  = sqrtf(vx_target * vx_target + vy_target * vy_target);
                    if (v_xy + vw_eq > 5.2f)
                    {
                        float scale = 1.0f / (v_xy + vw_eq);
                        vx_target *= scale;
                        vy_target *= scale;
                        vw_target *= scale;
                    }

                    string_target[0].TIM_Calculate_PeriodElapsedCallback(vx_target, string_fk.GetChassisVx()); // 斜坡规划 X方向（前后）
                    string_target[1].TIM_Calculate_PeriodElapsedCallback(vy_target, string_fk.GetChassisVy()); // 斜坡规划 Y方向（左右）
                    chassis_target.target_translation_x = string_target[0].GetOut();    // X方向期望（前后）
                    chassis_target.target_translation_y = string_target[1].GetOut();    // Y方向期望（左右）
                    chassis_target.target_rotation = vw_target;
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
        stringAngle_pid[i].reset();
        stringVelocity_pid[i].reset();
        // 输出置零
        chassis_output.out_string[i] = 0.0f;
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
    // 设置当前舵向角
    for(int i = 0; i < 4; i++)
    {
        string_ik.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
        string_fk.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
        string_id.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
    }
    // 正运动学解算当前底盘整体速度；逆运动学解算电机转速
    string_fk.StringForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    string_ik.StringInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 1.0f, 1.0f);
    
    for(int i = 0; i < 4; i++)
    {
        // 逆运动学相关 通过PID算舵向输出和轮向补偿
        stringAngle_pid[i].UpDate(string_ik.GetMotor_direction(i)*57.3f, Motor4005.getAngleDeg(i+1));
        stringVelocity_pid[i].UpDate(stringAngle_pid[i].getOutput(), Motor4005.getVelocityRpm(i+1));

        wheel_pid[i].UpDate(string_ik.GetMotor_wheel(i), Motor3508.getVelocityRads(i+1));

        chassis_output.out_string[i] = stringVelocity_pid[i].getOutput();
        //chassis_output.out_wheel[i] = wheel_pid[i].getOutput();

        // 正运动学相关 通过PID算牵引力和旋转力矩
        translational_pid[0].UpDate(chassis_target.target_translation_x, string_fk.GetChassisVx());
        translational_pid[1].UpDate(chassis_target.target_translation_y, string_fk.GetChassisVy());
        rotational_pid.UpDate(chassis_target.target_rotation, string_fk.GetChassisVw());

        // 逆动力学 算轮向电机力矩
        string_id.StringInvDynamics(translational_pid[0].getOutput(), translational_pid[1].getOutput(), rotational_pid.getOutput());

        // 轮向电机力矩转控制电流
        chassis_output.out_wheel[i] = string_id.GetMotorTorque(i) / 15.76f / 0.7f / 0.3f * 819.2f + wheel_pid[i].getOutput();
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

    float I4005[4], I3508[4], I_other[4], V4005[4], V3508[4];   // 必要参数
    for(int i = 0; i < 4; i++)
    {
        I4005[i] = chassis_output.out_string[i] * 32.0f/2000.0f;    // 4005电机电流（解算欲输出电流）
        V4005[i] = Motor4005.getVelocityRads(i+1);              // 4005电机速度（当前速度）

        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;    // 3508电机电流（解算欲输出电流）
        I_other[i] = 0.0f;                                          // 3508电机电流（非解算欲输出电流）
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f); // 3508电机速度（当前速度）
    }
    float pmax4005 = PowerMax*0.5f; // 4005电机先吃50%
    float pmax3508 = PowerMax*0.5f; // 3508电机吃50%
    // if(pmax4005 > 50.0f) 这个可能没什么用，主要是想给4005少一点功率，放着先
    // {
    //     pmax4005 = 50.0f;
    //     pmax3508 = PowerMax - pmax4005;
    // }
    power4005.AttenuatedPower(I4005, V4005, coefficients4005, 0.0f, pmax4005);  // 4005电机功率控制（衰减功率法）
    float PowerTotal_4005 = power4005.getPowerTotal();  // 4005电机总功率
    if(PowerTotal_4005 < pmax4005)  // 4005电机没吃完50%
    {
        pmax3508 = PowerMax - PowerTotal_4005;  // 3508电机吃剩下的全部
    }
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508); // 3508电机功率控制（衰减电流法）

    // 底盘输出
    for(int i = 0; i < 4; i++)
    {
        // chassis_output.out_string[i] = power4005.getCurrentCalculate(i) * 2000.0f/32.0f;
        chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;
        chassis_output.out_string[i] = std::clamp(chassis_output.out_string[i], -2000.0f, 2000.0f);
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
}

/**
 * @brief 底盘跟随控制函数
 * 
 * 实现底盘跟随控制
 */
void chassis_follow()
{   
    // 设置当前舵向角
    for(int i = 0; i < 4; i++)
    {
        string_ik.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
        string_fk.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
        string_id.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
    }
    // 正运动学解算当前底盘整体速度；逆运动学解算电机转速
    string_fk.StringForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    string_ik.StringInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 1.0f, 1.0f);
    
    for(int i = 0; i < 4; i++)
    {
        // 逆运动学相关 通过PID算舵向输出和轮向补偿
        stringAngle_pid[i].UpDate(string_ik.GetMotor_direction(i)*57.3f, Motor4005.getAngleDeg(i+1));
        stringVelocity_pid[i].UpDate(stringAngle_pid[i].getOutput(), Motor4005.getVelocityRpm(i+1));

        wheel_pid[i].UpDate(string_ik.GetMotor_wheel(i), Motor3508.getVelocityRads(i+1));

        chassis_output.out_string[i] = stringVelocity_pid[i].getOutput();
        //chassis_output.out_wheel[i] = wheel_pid[i].getOutput();

        // 正运动学相关 通过PID算牵引力和旋转力矩
        translational_pid[0].UpDate(chassis_target.target_translation_x, string_fk.GetChassisVx());
        translational_pid[1].UpDate(chassis_target.target_translation_y, string_fk.GetChassisVy());
        rotational_pid.UpDate(chassis_target.target_rotation, string_fk.GetChassisVw());

        // 逆动力学 算轮向电机力矩
        string_id.StringInvDynamics(translational_pid[0].getOutput(), translational_pid[1].getOutput(), rotational_pid.getOutput());

        // 轮向电机力矩转控制电流
        chassis_output.out_wheel[i] = string_id.GetMotorTorque(i) / 15.76f / 0.7f / 0.3f * 819.2f + wheel_pid[i].getOutput();
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

    float I4005[4], I3508[4], I_other[4], V4005[4], V3508[4];   // 必要参数
    for(int i = 0; i < 4; i++)
    {
        I4005[i] = chassis_output.out_string[i] * 32.0f/2000.0f;    // 4005电机电流（解算欲输出电流）
        V4005[i] = Motor4005.getVelocityRads(i+1);                  // 4005电机速度（当前速度）

        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;    // 3508电机电流（解算欲输出电流）
        I_other[i] = 0.0f;                                          // 3508电机电流（非解算欲输出电流）
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f); // 3508电机速度（当前速度）
    }
    float pmax4005 = PowerMax*0.5f; // 4005电机先吃50%
    float pmax3508 = PowerMax*0.5f; // 3508电机吃50%
    // if(pmax4005 > 50.0f) 这个可能没什么用，主要是想给4005少一点功率，放着先
    // {
    //     pmax4005 = 50.0f;
    //     pmax3508 = PowerMax - pmax4005;
    // }
    power4005.AttenuatedPower(I4005, V4005, coefficients4005, 0.0f, pmax4005);  // 4005电机功率控制（衰减功率法）
    float PowerTotal_4005 = power4005.getPowerTotal();  // 4005电机总功率
    if(PowerTotal_4005 < pmax4005)  // 4005电机没吃完50%
    {
        pmax3508 = PowerMax - PowerTotal_4005;  // 3508电机吃剩下的全部
    }
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508); // 3508电机功率控制（衰减电流法）

    // 底盘输出
    for(int i = 0; i < 4; i++)
    {
        // chassis_output.out_string[i] = power4005.getCurrentCalculate(i) * 2000.0f/32.0f;
        chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;
        chassis_output.out_string[i] = std::clamp(chassis_output.out_string[i], -2000.0f, 2000.0f);
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
}

void chassis_navigation()
{
    // 设置当前舵向角
    for(int i = 0; i < 4; i++)
    {
        string_ik.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
        string_fk.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
        string_id.Set_current_steer_angles(Motor4005.getAngleRad(i+1), i);
    }
    // 正运动学解算当前底盘整体速度；逆运动学解算电机转速
    string_fk.StringForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    string_ik.StringInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 1.0f, 1.0f);
    
    for(int i = 0; i < 4; i++)
    {
        // 逆运动学相关 通过PID算舵向输出和轮向补偿
        stringAngle_pid[i].UpDate(string_ik.GetMotor_direction(i)*57.3f, Motor4005.getAngleDeg(i+1));
        stringVelocity_pid[i].UpDate(stringAngle_pid[i].getOutput(), Motor4005.getVelocityRpm(i+1));

        wheel_pid[i].UpDate(string_ik.GetMotor_wheel(i), Motor3508.getVelocityRads(i+1));

        chassis_output.out_string[i] = stringVelocity_pid[i].getOutput();
        //chassis_output.out_wheel[i] = wheel_pid[i].getOutput();

        // 正运动学相关 通过PID算牵引力和旋转力矩
        translational_pid[0].UpDate(chassis_target.target_translation_x, string_fk.GetChassisVx());
        translational_pid[1].UpDate(chassis_target.target_translation_y, string_fk.GetChassisVy());
        rotational_pid.UpDate(chassis_target.target_rotation, string_fk.GetChassisVw());

        // 逆动力学 算轮向电机力矩
        string_id.StringInvDynamics(translational_pid[0].getOutput(), translational_pid[1].getOutput(), rotational_pid.getOutput());

        // 轮向电机力矩转控制电流
        chassis_output.out_wheel[i] = string_id.GetMotorTorque(i) / 15.76f / 0.7f / 0.3f * 819.2f + wheel_pid[i].getOutput();
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

    float I4005[4], I3508[4], I_other[4], V4005[4], V3508[4];   // 必要参数
    for(int i = 0; i < 4; i++)
    {
        I4005[i] = chassis_output.out_string[i] * 32.0f/2000.0f;    // 4005电机电流（解算欲输出电流）
        V4005[i] = Motor4005.getVelocityRads(i+1);                  // 4005电机速度（当前速度）

        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;    // 3508电机电流（解算欲输出电流）
        I_other[i] = 0.0f;                                          // 3508电机电流（非解算欲输出电流）
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f); // 3508电机速度（当前速度）
    }
    float pmax4005 = PowerMax*0.5f; // 4005电机先吃50%
    float pmax3508 = PowerMax*0.5f; // 3508电机吃50%
    // if(pmax4005 > 50.0f) 这个可能没什么用，主要是想给4005少一点功率，放着先
    // {
    //     pmax4005 = 50.0f;
    //     pmax3508 = PowerMax - pmax4005;
    // }
    power4005.AttenuatedPower(I4005, V4005, coefficients4005, 0.0f, pmax4005);  // 4005电机功率控制（衰减功率法）
    float PowerTotal_4005 = power4005.getPowerTotal();  // 4005电机总功率
    if(PowerTotal_4005 < pmax4005)  // 4005电机没吃完50%
    {
        pmax3508 = PowerMax - PowerTotal_4005;  // 3508电机吃剩下的全部
    }
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508); // 3508电机功率控制（衰减电流法）

    // 底盘输出
    for(int i = 0; i < 4; i++)
    {
        // chassis_output.out_string[i] = power4005.getCurrentCalculate(i) * 2000.0f/32.0f;
        chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;
        chassis_output.out_string[i] = std::clamp(chassis_output.out_string[i], -2000.0f, 2000.0f);
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
}

void NavigationParser()
{
    static constexpr float dt = 0.001f; // 控制周期1ms

    navigation.Vx = string_fk_odometer_v.GetChassisVx();
    navigation.Vy = string_fk_odometer_v.GetChassisVy();
    navigation.Wz = string_fk_odometer_v.GetChassisVw();

    navigation.X -= navigation.Vx * dt;
    navigation.Y -= navigation.Vy * dt;
    navigation.Yaw += navigation.Wz * dt;
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
        case NAVIGATION: 
            chassis_navigation();
            break;
        default:
            chassis_stop();
            break;
    }

    // 里程计
    for (int i = 0; i < 4; i++)
    {
        string_fk_odometer_v.Set_current_steer_angles(Motor4005.getAngleRad(i + 1), i);
    }
    string_fk_odometer_v.StringForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    NavigationParser();
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
        // 暂时不使用蜂鸣器
        // BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online(), alphabet);
        for(int i = 0; i < 4; i++)
        {
            target_4005_angle[i] = string_ik.GetMotor_direction(i)*57.3f;
        }
        osDelay(1);
    } 
}
}