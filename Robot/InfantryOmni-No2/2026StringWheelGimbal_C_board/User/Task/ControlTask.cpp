#include "ControlTask.hpp"
#include <algorithm>

/* 外部变量 ------------------------------------------------------------------------------------------------*/
extern bool alphabet[28];
extern bool is_change;
extern bool is_vision;

/**
 * @brief 初始化
 */
/* 有限状态机 -----------------------------------------------------------------------------------------------*/
Gimbal_FSM gimbal_fsm;  // 云台
Launch_FSM launch_fsm;  // 发射机构

/* 滤波器 ---------------------------------------------------------------------------------------------------*/
TDFilter vision_filter_pitch(30.0f, 0.005f);    // 视觉pitch滤波器
TDFilter vision_filter_yaw(40.0f, 0.005f);      // 视觉yaw滤波器
TDFilter target_yaw(30.0f, 0.005f);     // 键鼠模式的yaw轴目标滤波
TDFilter ude_filter_input(40.0f, 0.005f);   // UDE滤波器,用于获取差分信号
TDFilter ude_filter_out(20.0f, 0.001f);   // UDE滤波器，用于最终输出滤波

/* 前馈 -----------------------------------------------------------------------------------------------------*/
Alg::Feedforward::Friction friction_forward(200.0f);        // 摩擦力前馈（普通模式 Yaw）
Alg::Feedforward::Gravity gravity_forward(-0.85f, 115.0f);  // 重力前馈（所有模式 Pitch）
Alg::Feedforward::Acceleration acc_forward(0.5f, 0.005f);   // 加速度前馈（普通模式 Yaw）
Alg::Feedforward::Velocity velocity_forward(0.5f, 0.005f);  // 速度前馈（视觉模式 Yaw）
Alg::Feedforward::GimbalFullCompensation gimbal_yaw(0.005f, 0.001f, 0.005369f, 0.032416f);  // k_J初始为0，控制周期1ms
Alg::Feedforward::UDE ude_yaw(-0.39f, 6.6f);  // UDE前馈

/* 控制器 ---------------------------------------------------------------------------------------------------*/
//ALG::PID::PID yaw_pid(100.0f, 0.0f, 0.0f, 16384.0f, 30.0f, 3.0f);  // yaw轴速控pid（普通模式）
ALG::PID::PID yaw_pid(500.0f, 0.0f, 1200.0f, 16384.0f, 30.0f, 3.0f);  // yaw轴速控pid（普通模式）
ALG::PID::PID yaw_angle_pid(2.0f, 0.0f, 7.5f, 16384.0f, 0.0f, 0.0f);        // yaw轴角速度pid（视觉模式）
ALG::PID::PID yaw_velocity_pid(400.0f, 0.0f, 0.0f, 16384.0f, 0.0f, 0.0f);   // yaw轴角速度pid（视觉模式）

ALG::PID::PID pitch_manual_pid(60.0f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f); // pitch轴pid（普通模式）用于设置内置pid的 KP,KD
ALG::PID::PID pitch_vision_pid(60.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f);  // pitch轴pid（视觉模式）用于设置内置pid的 KP,KD

ALG::PID::PID dial_pid(6.0f, 0.0f, 0.0f, 10000.0f, 2500.0f, 200.0f);    // 拨盘速控pid （停火、急停模式）
ALG::PID::PID dial_angle_pid(4.0f, 0.0f, 0.0f, 10000.0f, 2500.0f, 200.0f);     // 拨盘角速度pid （单、连发模式）
ALG::PID::PID dial_velocity_pid(4.5f, 0.0f, 0.0f, 10000.0f, 2500.0f, 200.0f);  // 拨盘角速度pid （单、连发模式）

ALG::PID::PID surgewheel_pid[2] = {             
    ALG::PID::PID(10.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f), //摩擦轮速控pid（左）
    ALG::PID::PID(10.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f)  //摩擦轮速控pid（右）
};

/* 热量控制 --------------------------------------------------------------------------------------------------*/
APP::Heat_Control_Private heat_control(100, 50, 30.0f, 1.0f, 0.001f);

/* 期望值与输出 ----------------------------------------------------------------------------------------------*/
ControlTask gimbal_target;      // 云台与发射机构期望值

Output_gimbal gimbal_output;    // 云台输出
Output_launch launch_output;    // 发射机构输出


/**
 * @brief 云台与发射机构控制逻辑
 */
/* 公共部分 ---------------------------------------------------------------------------------------------------*/

/**
 * @brief 检查所有关键设备是否在线
 * 
 * @return true 设备全部在线
 * @return false 存在离线设备
 */
bool check_online()
{
    bool isconnected = true;
    // 上场要注释掉。不然发射机构掉电云台会失能，过不了检录。
    // if(!Motor6020.isConnected(1, 6) || !MotorJ4310.isConnected(1, 4) || !Motor3508.isConnected(1, 2) || !Motor3508.isConnected(1, 3) || !Motor3508.isConnected(1, 1))
    // {
    //     isconnected = false;
    // }

    if(!DT7.isConnected() || !HI12.isConnected())
    {
        isconnected = false;
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
    gimbal_fsm.Init();
    launch_fsm.Init();
}

/**
 * @brief 计算pitch轴目标角速度
 * 
 * @param this_target 目标角度（单位 度）
 * @param control_cycle 控制周期（单位 s）
 * @return float 目标角速度（单位 rad/s）
 */
float CalculatePitchVel(float this_target, float control_cycle)
{
    static float last_target_pitch = this_target; 
    float pitch_vel_deg_s = (this_target - last_target_pitch) / control_cycle;
    float pitch_vel_rad_s = pitch_vel_deg_s * 3.1415926f / 180.0f;
    last_target_pitch = this_target;  
    return pitch_vel_rad_s;
}

/**
 * @brief 赫兹转角度
 * 
 * @param fire_hz 
 * @return float 
 */
float hz_to_angle(float fire_hz)
{
    const int slots_per_rotation = 9;                         // 拨盘一圈9个弹
    const float angle_per_slot = 360.0f / slots_per_rotation; // 360/9 = 40度 (出一个蛋需要的角度)
    const float control_period = 0.005f;                      // 5ms (200Hz的控制频率)
    // 公式推导：
    // 1. 每秒需要出的蛋数 = fire_hz
    // 2. 每秒需要转过的角度 = fire_hz * 40度
    // 3. 每5毫秒(每帧)需要转过的角度 = (fire_hz * 40度) / 200
    
    float angle_per_frame = (fire_hz * angle_per_slot * 36.0f) * control_period;
    return angle_per_frame;
}

/**
 * @brief 设置云台和发射机构的目标值
 * 
 * 根据当前工作模式设置相应的目标值
 */
void SetTarget()
{
    static int last_launch_state = -1;
    // 设置死区
    DT7.SetDeadzone(20.0f);
    // 云台
    switch(gimbal_fsm.Get_Now_State()) 
    {
        case STOP:      // 停止模式
            gimbal_target.target_yaw = 0.0f;
            gimbal_target.target_pitch = MotorJ4310.getAngleDeg(1);
            break;
        case MANUAL:    // 普通模式
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)
            {
                gimbal_target.target_yaw = target_yaw.filter(0.3f * -DT7.get_mouseX());
                
                // 获取鼠标原始值
                float mouse_y_input = DT7.get_mouseY();
                // 异常脉冲/脏数据过滤 如果一帧的鼠标移动量超过 200 那绝对是解析错位或者是乱码脏数据，直接丢弃这一帧
                if (fabsf(mouse_y_input) > 200.0f) 
                {
                    mouse_y_input = 0.0f; 
                }
                // 去除零漂
                if (fabsf(mouse_y_input) < 1.0f) 
                {
                    mouse_y_input = 0.0f;
                }
                // 灵敏度
                float pitch_sensitivity = 0.004f; 
                float angle_delta = mouse_y_input * pitch_sensitivity;

                gimbal_target.target_pitch -= angle_delta;
                gimbal_target.target_pitch = std::clamp(gimbal_target.target_pitch, -20.0f, 20.0f);
            }
            else
            {
                gimbal_target.target_yaw = 50.0f * -DT7.get_right_x();  // 速度rpm
                //gimbal_target.target_yaw = SinExpected(0.001f, 20, 90, 4);
                gimbal_target.target_pitch -= DT7.get_right_y();        // 角度deg
                gimbal_target.target_pitch = std::clamp(gimbal_target.target_pitch, -20.0f, 20.0f);  // 角度deg
            }
            break;
        case VISION:    // 视觉模式
            gimbal_target.target_yaw = -vision.getTarYaw();        // 角度deg TD滤波
            gimbal_target.target_pitch = vision.getTarPitch();  // 角度deg TD滤波
            gimbal_target.target_pitch = std::clamp(gimbal_target.target_pitch, -20.0f, 20.0f);  // 角度deg
            break;
        default:
            gimbal_target.target_yaw = 0.0f;
            gimbal_target.target_pitch = MotorJ4310.getAngleDeg(1);
            break;
    }
    // 发射机构
    switch(launch_fsm.Get_Now_State()) 
    {
        case LAUNCH_STOP:       // 停止模式
            gimbal_target.target_dial = 0.0f;
            gimbal_target.target_surgewheel[0] = 0.0f;
            gimbal_target.target_surgewheel[1] = 0.0f;
            break;
        case LAUNCH_CEASEFIRE:  // 停火模式
            gimbal_target.target_dial = 0.0f;
            gimbal_target.target_surgewheel[0] = 6000.0f;
            gimbal_target.target_surgewheel[1] = -6000.0f;
            break;
        case LAUNCH_ONLY:
            // 进状态时同步一次目标值
            if (last_launch_state != LAUNCH_ONLY) 
            {
                gimbal_target.target_dial = Motor3508.getAddAngleDeg(1);
            }
            
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)
            {
                static bool last_mouse_left = false;
                if (alphabet[26] && !last_mouse_left)   // 上升沿检测
                {
                    if(heat_control.GetNowFire() > 0) 
                    {
                        gimbal_target.target_dial -= 40.0f*36.0f; // 只减一次
                    }
                }
                last_mouse_left = alphabet[26];
                
                gimbal_target.target_surgewheel[0] = 6000.0f;
                gimbal_target.target_surgewheel[1] = -6000.0f;
            }
            else
            {
                static bool last_scroll_active = false;
                
                // 阈值判断
                float scroll_val = DT7.get_scroll_();
                bool current_scroll_active = (fabs(scroll_val) > 0.8f);

                // 下降沿检测：从 高位 变 低位
                if (last_scroll_active && !current_scroll_active)
                {
                    if(heat_control.GetNowFire() > 0) 
                    {
                        gimbal_target.target_dial -= 40.0f*36.0f; // 只减一次
                    }
                    else
                    {
                        gimbal_target.target_dial += 0.0f;  
                    }
                }
                last_scroll_active = current_scroll_active;

                gimbal_target.target_surgewheel[0] = 6000.0f;
                gimbal_target.target_surgewheel[1] = -6000.0f;
            }
            break;
        case LAUNCH_AUTO:
            if (last_launch_state != LAUNCH_AUTO) 
            {
                gimbal_target.target_dial = Motor3508.getAddAngleDeg(1);
            }
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)
            {
                gimbal_target.target_surgewheel[0] = 6000.0f;
                gimbal_target.target_surgewheel[1] = -6000.0f;
                gimbal_target.target_dial -= DT7.get_mouseLeft() * hz_to_angle(heat_control.GetNowFire());
                //gimbal_target.target_dial -= DT7.get_mouseLeft() * hz_to_angle(13.0f);
            }
            else
            {
                gimbal_target.target_surgewheel[0] = 6000.0f;
                gimbal_target.target_surgewheel[1] = -6000.0f;
                gimbal_target.target_dial -= DT7.get_scroll_() * hz_to_angle(heat_control.GetNowFire());
                //gimbal_target.target_dial -= DT7.get_scroll_() * hz_to_angle(13.0f);
            }
            break;
        case LAUNCH_JAM:
            gimbal_target.target_dial = 5000; // 直接给控制电流开环
            gimbal_target.target_surgewheel[0] = 6000.0f;
            gimbal_target.target_surgewheel[1] = -6000.0f;
            break;
        default:  // 默认模式（停止）
            gimbal_target.target_dial = 0.0f;
            gimbal_target.target_surgewheel[0] = 0.0f;
            gimbal_target.target_surgewheel[1] = 0.0f;
            break;
    }

    // 更新 Pitch 轴目标速度 (必须全程运行以保证微分连续性)
    gimbal_target.target_pitch_vel = CalculatePitchVel(gimbal_target.target_pitch, 0.005f); // 速度rad/s

    // 视觉模式下不使用速度前馈
    if(gimbal_fsm.Get_Now_State() == VISION)
    {
        gimbal_target.target_pitch_vel = 0.0f;
    }
    last_launch_state = launch_fsm.Get_Now_State();
}

/* 云台部分 ---------------------------------------------------------------------------------------------------*/

/**
 * @brief 云台停止模式控制函数
 * 
 * 关闭电机使能并重置控制器参数
 */
void gimbal_stop()
{
    // 失能4310
    if(MotorJ4310.getIsenable())
    {
        MotorJ4310.Off(1, BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(false);
    }
    // 重置控制器
    yaw_pid.reset();
    yaw_angle_pid.reset();
    yaw_velocity_pid.reset();
    // 输出置零
    gimbal_output.out_yaw = 0.0f;
    gimbal_output.out_pitch = 0.0f;
}

/**
 * @brief 云台普通模式控制函数
 * 
 * 实现手动遥控控制云台运动
 */
void gimbal_manual()
{
    // 4310使能：如果达妙电机硬件反馈状态不是1（使能态），就一直发唤醒帧
    if(MotorJ4310.GetErr(1) != 1)
    {
        MotorJ4310.On(1, BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(true);
    }

    // Yaw 速控PID
    yaw_pid.UpDate(gimbal_target.target_yaw, HI12.GetGyroRPM(2));
    // 模型前馈
    gimbal_yaw.MomentOfInertiaTuning(HI12.GetGyroRPM(2), gimbal_target.target_yaw);
    // 在有期望值才加模型前馈，避免零飘，gimbal_yaw.getFriction() 输出单位是Nm，需要转化成控制电流
    float model_comp = (fabsf(gimbal_target.target_yaw) > 1.0f) ? 0.3f * gimbal_yaw.getFriction() / 0.741f * (16384.0f / 3.0f) : 0.0f;
    
    // UDE估计器
    static float last_total_out_yaw = 0.0f; // 由于因果律，需获取上一次的输出
    float ude_input = last_total_out_yaw * (3.0f / 16384.0f); // 将控制电流转化为真是电流
    ude_filter_input.filter(HI12.GetGyroRad(2));    // 获取X_dot，使用TD获取微分
    ude_yaw.UDE_Update(ude_input, ude_filter_input.getDerivative());    // UDE计算
    float ude_output = (ude_filter_out.filter(ude_yaw.getOutput())) * 16384.0f/3.0f;    // UDE输出转化为控制电流
    // 使用线性衰减来限制UDE的补偿，防止云台在急速移动时UDE补偿过大
    float speed_error = fabsf(gimbal_target.target_yaw - HI12.GetGyroRPM(2)); // 计算期望速度与实际速度的误差
    float k_ude = 1.0f; // 系数
    if(speed_error > 20.0f) 
    {
        k_ude = 1.0f - (speed_error - 20.0f) / 80.0f; // 线性衰减
    }
    k_ude = std::clamp(k_ude, 0.0f, 1.0f); // 保证系数在0到1之间
    // 只采信安全平缓状态下的UDE补偿
    float ude_output_safe = ude_output * k_ude;
    // 小陀螺才开启UDE补偿
    bool scroll = (gimbal_fsm.Get_Now_State() == MANUAL) && (launch_fsm.Get_Now_State() == LAUNCH_AUTO || launch_fsm.Get_Now_State() == LAUNCH_ONLY || launch_fsm.Get_Now_State() == LAUNCH_JAM);
    if((!scroll && (DT7.get_scroll_() != 0.0f)) || alphabet[23])
    {
        gimbal_output.out_yaw = yaw_pid.getOutput() + model_comp - ude_output_safe;
    }
    else
    {
        gimbal_output.out_yaw = yaw_pid.getOutput() + model_comp;
    }
    // Yaw 输出，进行限幅
    gimbal_output.out_yaw = std::clamp(gimbal_output.out_yaw, -16384.0f, 16384.0f);
    last_total_out_yaw = gimbal_output.out_yaw; // 更新上一时刻的yaw输出，UDE用
    
    // Pitch 重力补偿； Pitch控制用内置pid
    gravity_forward.GravityFeedforward(MotorJ4310.getAngleDeg(1));
    float gravity_comp = gravity_forward.getFeedforward();
    // Pitch 输出（放入MIT的扭矩项）
    gimbal_output.out_pitch = gravity_comp;
}

/**
 * @brief 云台视觉模式控制函数
 * 
 * 实现视觉引导的云台控制
 */
void gimbal_vision()
{
    // 4310使能
    if(MotorJ4310.GetErr(1) != 1)
    {
        MotorJ4310.On(1, BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(true);
    }

    // Yaw 角速度PID
    float current_yaw = HI12.GetAngle(2);
    float err_yaw = gimbal_target.target_yaw - current_yaw;
    
    // 过零处理，保证走最短路径 (适用于[-180, 180]的突变)
    while (err_yaw > 180.0f)  err_yaw -= 360.0f;
    while (err_yaw < -180.0f) err_yaw += 360.0f;
    
    yaw_angle_pid.UpDate(current_yaw + err_yaw, current_yaw);
    yaw_velocity_pid.UpDate(yaw_angle_pid.getOutput(), HI12.GetGyroRPM(2));
    // 速度前馈补偿
    velocity_forward.VelocityFeedforward(gimbal_target.target_yaw);
    float velocity_comp = velocity_forward.getFeedforward() / 6.0f; // 从 deg/s 转为 rpm
    // Yaw 输出
    gimbal_output.out_yaw = yaw_velocity_pid.getOutput() + velocity_comp;

    // Pitch 重力补偿； Pitch控制用内置pid
    gravity_forward.GravityFeedforward(MotorJ4310.getAngleDeg(1));
    float gravity_comp = gravity_forward.getFeedforward();
    // Pitch 输出（放入MIT的扭矩项）
    gimbal_output.out_pitch = gravity_comp;
}

/**
 * @brief 云台主控制循环
 * 
 * @param left_sw 遥控器左开关状态
 * @param right_sw 遥控器右开关状态
 * @param is_online 设备在线状态
 */
void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online) 
{   
    static int last_gimbal_state = -1;
    gimbal_fsm.StateUpdate(left_sw, right_sw, is_online, vision.getVisionFlag());
    
    // 状态机发生切换
    if (gimbal_fsm.Get_Now_State() != last_gimbal_state)
    {
        // 跨越模式时清空陈旧的积分（I）和微分（D）误差，防止参数跳变导致“疯头”
        yaw_pid.reset();
        yaw_angle_pid.reset();
        yaw_velocity_pid.reset();
        
        last_gimbal_state = gimbal_fsm.Get_Now_State();
    }

    SetTarget();

    switch(gimbal_fsm.Get_Now_State()) 
    {
        case STOP:      // 停止模式
            gimbal_stop();
            break;
        case MANUAL:    // 普通模式
            gimbal_manual();
            break;
        case VISION:    // 视觉模式
            gimbal_vision();
            break;
        default:        // 默认模式（停止）
            gimbal_stop();
            break;
    }
}

/* 发射机构部分 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 发射机构停止模式控制函数
 * 
 * 重置所有控制器并设置输出为0
 */
void launch_velocity()
{
    // 速度环期望值置零，快速停下
    dial_pid.UpDate(gimbal_target.target_dial, Motor3508.getVelocityRpm(1));
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].UpDate(gimbal_target.target_surgewheel[i], Motor3508.getVelocityRpm(i + 2));
    }
    // 输出
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
    launch_output.out_dial = dial_pid.getOutput();
}

/**
 * @brief 发射机构单发模式控制函数
 * 
 * 拨盘按角度控制，摩擦轮保持设定转速
 */
void launch_angle()
{
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].UpDate(gimbal_target.target_surgewheel[i], Motor3508.getVelocityRpm(i + 2));
    }
    // 输出
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();

    // 拨盘角度环
    dial_angle_pid.UpDate(gimbal_target.target_dial, Motor3508.getAddAngleDeg(1));
    dial_velocity_pid.UpDate(dial_angle_pid.getOutput(), Motor3508.getVelocityRpm(1));
    launch_output.out_dial = dial_velocity_pid.getOutput();
}

/**
 * @brief 发射机构卡弹控制函数
 * 
 * 卡弹的时候输出反向力矩
 */
void launch_jam()
{
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].UpDate(0.0f, Motor3508.getVelocityRpm(i + 2));
    }
    // 输出
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
    launch_output.out_dial = gimbal_target.target_dial;
}

/**
 * @brief 发射机构主控制循环
 * 
 */
void main_loop_launch()
{
    SetTarget();

    // 基于 Action 和 Mode 执行控制函数
    switch(launch_fsm.Get_Now_State())
    {
        case LAUNCH_STOP:
            launch_velocity();
            break;
        case LAUNCH_CEASEFIRE:
            launch_velocity();
            break;
        case LAUNCH_AUTO:
            launch_angle();
            break;
        case LAUNCH_ONLY:
            launch_angle();
            break;
        case LAUNCH_JAM:
            launch_jam();
            break;
        default:
            launch_velocity();
            break;
    }
}

/**
 * @brief 卡弹检测
 * 
 * @return true 
 * @return false 
 */
bool is_jamming()
{
    static uint32_t last_check_time = 0;
    static float last_angle = 0;
    
    float current_angle = Motor3508.getAddAngleDeg(1);
    float err = fabs(gimbal_target.target_dial - current_angle);
    
    // 1. 位置误差超过 120 度
    if (err > 120.0f * 36.0f)
    {
        uint32_t now = HAL_GetTick();
        
        // 初始化
        if (last_check_time == 0)
        {
            last_check_time = now;
            last_angle = current_angle;
            return false;
        }
        
        // 2. 每 300ms 检查一次
        if (now - last_check_time > 300)
        {
            // 3. 如果这 300ms 内转动角度小于 120 度 -> 判定卡弹
            if (fabs(current_angle - last_angle) < 120.0f * 36.0f)
            {
                // 重置检测状态（因为即将进入 Jam 模式，Jam模式下 error 会变小）
                last_check_time = 0; 
                return true;
            }
            
            // 更新基准
            last_check_time = now;
            last_angle = current_angle;
        }
    }
    else
    {
        // 误差不大的时候，重置计时器
        last_check_time = 0;
    }
    
    return false;
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
    // 初始化蜂鸣器管理器；初始化4310到失能；初始化状态机
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    MotorJ4310.Off(0x01, BSP::Motor::DM::MIT);
    fsm_init();

    static uint8_t control_tick = 0;
    for(;;)
    {
        // 更新蜂鸣器管理器，处理队列中的响铃请求
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        // 达妙电机（J4310）掉电离线时，自动清除软件使能标志位。
        // 这样在 24V 恢复供电复活时，gimbal_manual() 就会重新发送 On() 唤醒电机。
        // if (!MotorJ4310.isConnected(1, 4))
        // {
        //     MotorJ4310.setIsenable(false);
        // }
        
        // 1kHz 采样 (捕捉信号)
        
        // 热量控制
        float current[2] = {Motor3508.getCurrent(2), Motor3508.getCurrent(3)};
        float velocity[2] = {Motor3508.getVelocityRpm(2), Motor3508.getVelocityRpm(3)}; 
        heat_control.HeatControl(gimbal_target.target_surgewheel[0], current, velocity, Cboard.GetHeatLimit(), Cboard.GetHeatCool(), 0.001f, 15.0f);

        // 1000Hz转200Hz
        control_tick++;
        if (control_tick >= 5)
        {
            control_tick = 0;
            
            // 云台与发射机构主控制循环
            main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online());
            // 状态机更新 (使用 shoot 变量)
            launch_fsm.StateUpdate(DT7.get_s1(), DT7.get_s2(), check_online(), is_change, 4000.0f, vision.getVisionMode(), is_vision, heat_control.GetShot(), is_jamming(), alphabet);
            main_loop_launch();
        }
        
        osDelay(1);
    } 
}
}

