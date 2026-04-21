#include "ControlTask.hpp"
#include <algorithm>

/* 外部变量 ------------------------------------------------------------------------------------------------*/
extern bool alphabet[28];
extern bool is_change;
extern bool is_vision;

// 偏移角度
namespace
{
    constexpr float kTransformYawTargetPrimaryDeg = -90.5f;
    constexpr float kTransformYawTargetAlternateDeg = 89.5f;
    constexpr float kYawEquivalentSwitchHysteresisDeg = 8.0f;
}

/**
 * @brief 初始化
 */
/* 有限状态机 -----------------------------------------------------------------------------------------------*/
Gimbal_FSM gimbal_fsm;  // 云台
Launch_FSM launch_fsm;  // 发射机构

/* 斜坡规划 -------------------------------------------------------------------------------------------------*/


/* 滤波器---------------------------------------------------------------------------------------------------*/
TDFilter vision_filter_pitch(30.0f, 0.005f);    // 视觉pitch滤波
TDFilter vision_filter_yaw(40.0f, 0.005f);      // 视觉yaw滤波
TDFilter target_yaw_vel(60.0f, 0.005f);         // 键鼠模式的yaw轴目标滤
TDFilter target_pitch_vel(60.0f, 0.005f);
TDFilter ude_filter_input(40.0f, 0.005f);       // UDE滤波用于获取差分信号
TDFilter ude_filter_out(20.0f, 0.001f);         // UDE滤波器，用于最终输出滤

TDFilter vmc_pitch1_filter(15.0f, 0.005f);       // J1速度规划
TDFilter vmc_pitch2_filter(25.0f, 0.005f);      // J2速度规划
TDFilter vmc_yaw_filter(11.0f, 0.005f);          // Yaw 速度规划

/* 前馈 -----------------------------------------------------------------------------------------------------*/
Alg::Feedforward::Friction friction_forward(200.0f);        // 摩擦力前馈（普通模�?Yaw�?
Alg::Feedforward::Gravity gravity_forward(-0.85f, 115.0f);  // 重力前馈（所有模�?Pitch�?
Alg::Feedforward::Acceleration acc_forward(0.5f, 0.005f);   // 加速度前馈（普通模�?Yaw�?
Alg::Feedforward::Velocity velocity_forward(0.5f, 0.005f);  // 速度前馈（视觉模�?Yaw�?
Alg::Feedforward::GimbalFullCompensation gimbal_yaw(0.0f, 0.005f, 0.038062f, 0.120782f); // 全云台前馈
Alg::Feedforward::UDE ude_yaw(-0.39f, 6.6f);  // UDE前馈

Alg::Feedforward::Gravity gravity_forward_pitch1(0.0f, 0.0f);   // pitch1重补
Alg::Feedforward::Gravity gravity_forward_pitch2(0.0f, 0.0f);   // pitch2重补

/* 控制器 ---------------------------------------------------------------------------------------------------*/
ALG::ADRC::FirstLADRC yaw_adrc(25.0f, 70.0f, 105.0f, 0.005f, 800.0f, 3.0f);     // 普通模式Yaw 速控ADRC
ALG::ADRC::FirstLADRC yaw_adrc_try(40.0f, 40.0f, 800.0f, 0.005f, 450.0f, 3.0f); 
ALG::PID::PID yaw_transform_angle_pid(13.0f, 1.0f, 0.0f, 16384.0f, 30.0f, 5.0f);  // Yaw轴 变形角速度pid
ALG::PID::PID yaw_transform_velocity_pid(45.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 10.0f);

ALG::VMC::VMC doublePitch(0.14f, 0.0f, 2000.0f, 900.0f, 100.0f, 35.0f); // Pitch1 VMC

ALG::ADRC::FirstLADRC pitch2_adrc(50.0f, 165.0f, 400.0f, 0.005f, 450.0f, 3.0f);
ALG::PID::PID pitch2_transform_angle_pid(13.0f, 1.0f, 0.0f, 16384.0f, 30.0f, 5.0f);
ALG::PID::PID pitch2_transform_velocity_pid(35.0f, 1.0f, 0.0f, 16384.0f, 2500.0f, 500.0f);

ALG::PID::PID dial_pid(6.0f, 0.0f, 0.0f, 10000.0f, 2500.0f, 200.0f);    // 拨盘速控pid （停火、急停模式�?
ALG::PID::PID dial_angle_pid(4.0f, 0.0f, 0.0f, 10000.0f, 2500.0f, 200.0f);     // 拨盘角速度pid （单、连发模式）
ALG::PID::PID dial_velocity_pid(4.5f, 0.0f, 0.0f, 10000.0f, 2500.0f, 200.0f);  // 拨盘角速度pid （单、连发模式）

ALG::PID::PID surgewheel_pid[2] = {             
    ALG::PID::PID(10.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f), //摩擦轮速控pid（左�?
    ALG::PID::PID(10.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f)  //摩擦轮速控pid（右�?
};

/* 热量控制 --------------------------------------------------------------------------------------------------*/
APP::Heat_Control_Private heat_control(100, 50, 30.0f, 1.0f, 0.001f);

/* 期望值与输出 ----------------------------------------------------------------------------------------------*/
ControlTask gimbal_target;      // 云台与发射机构期望�?

Output_gimbal gimbal_output;    // 云台输出
Output_launch launch_output;    // 发射机构输出

/**
 * @brief 云台与发射机构控制逻辑
 */
/* 公共部分 ---------------------------------------------------------------------------------------------------*/

/**
 * @brief 获取标准误差
 * 
 * @param raw_target 输入目标
 * @param current_feedback 当前反馈
 * @return float 标准误差
 */
float getYawTargetProcessed(float raw_target, float current_feedback) 
{
    // 1. 计算标准误差
    float error = raw_target - current_feedback;

    // 2. 寻找最短路径误差
    // 这里的逻辑是：如果误差超过 180 度，说明跨越了跳变点
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;

    // 3. 计算“虚拟期望”
    // 虚拟期望 = 当前反馈 + 最短路径误差
    // 这样 PID 内部做 (VirtualTarget - Feedback) 时，结果正好等于最短误差
    float virtual_target = current_feedback + error;

    return virtual_target;
}

float selectYawEquivalentTarget(float raw_target, float current_feedback)
{
    bool is_transform_yaw_target =
        fabsf(raw_target - kTransformYawTargetPrimaryDeg) < 1.0f ||
        fabsf(raw_target - kTransformYawTargetAlternateDeg) < 1.0f;

    if (!is_transform_yaw_target)
    {
        return raw_target;
    }

    static float last_selected_target = kTransformYawTargetPrimaryDeg;
    float primary_error = fabsf(kTransformYawTargetPrimaryDeg - current_feedback);
    float alternate_error = fabsf(kTransformYawTargetAlternateDeg - current_feedback);

    if (fabsf(last_selected_target - kTransformYawTargetPrimaryDeg) < 1.0f)
    {
        if (alternate_error + kYawEquivalentSwitchHysteresisDeg < primary_error)
        {
            last_selected_target = kTransformYawTargetAlternateDeg;
        }
        else
        {
            last_selected_target = kTransformYawTargetPrimaryDeg;
        }
    }
    else
    {
        if (primary_error + kYawEquivalentSwitchHysteresisDeg < alternate_error)
        {
            last_selected_target = kTransformYawTargetPrimaryDeg;
        }
        else
        {
            last_selected_target = kTransformYawTargetAlternateDeg;
        }
    }

    return last_selected_target;
}

/**
 * @brief 检查所有关键设备是否在线
 * 
 * @return true 设备全部在线
 * @return false 存在离线设备
 */
bool check_online()
{
    bool isconnected = true;

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
 * @brief 赫兹转角度
 * 
 * @param fire_hz 
 * @return float 
 */
float hz_to_angle(float fire_hz)
{
    const int slots_per_rotation = 9;
    const float angle_per_slot = 360.0f / slots_per_rotation;
    const float control_period = 0.005f;

    float angle_per_frame = (fire_hz * angle_per_slot * 36.0f) * control_period;
    return angle_per_frame;
}


/**
 * @brief 云台目标
 */
void SetTarget_Gimbal()
{
    static Enum_Gimbal_States last_state = STOP;
    Enum_Gimbal_States now_state = gimbal_fsm.Get_Now_State();

    // 当前pitch2世界坐标系角度
    if(last_state == TRANSFORM && now_state == MANUAL)
    {
        gimbal_target.target_pitch2_angle = HI12.GetAngle(1);
    }
    if(last_state != now_state)
    {
        vmc_pitch1_filter.reset(-MotorJ4340.getAngleDeg(1) - 103.0f);
        vmc_pitch2_filter.reset(HI12.GetAngle(1));
        vmc_yaw_filter.reset(MotorJ4310.getAngleDeg(2));
    }
    last_state = now_state;

    DT7.SetDeadzone(20.0f);
    switch (now_state)
    {
        case STOP:
            gimbal_target.target_yaw_vel = 0.0f;
            gimbal_target.target_yaw_angle = MotorJ4310.getAngleDeg(2);
            gimbal_target.target_pitch1 = -MotorJ4340.getAngleDeg(1) - 103.0f;  // pitch1 转化坐标系后反馈角
            gimbal_target.target_pitch2_vel = 0.0f;
            gimbal_target.target_pitch2_angle = HI12.GetAngle(1);
            break;

        case MANUAL:
        {
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)
            {
                gimbal_target.target_yaw_vel = target_yaw_vel.filter(0.03f * -DT7.get_mouseX());
                gimbal_target.target_pitch1 = 0.0f;
                
                // 奇异信号处理
                float mouse_y_input = DT7.get_mouseY();
                if (fabsf(mouse_y_input) > 200.0f)
                {
                    mouse_y_input = 0.0f;
                }
                if (fabsf(mouse_y_input) < 1.0f)
                {
                    mouse_y_input = 0.0f;
                }

                gimbal_target.target_pitch2_vel = 0.02f * target_pitch_vel.filter(mouse_y_input);
                gimbal_target.target_pitch2_angle = HI12.GetAngle(1);
            }
            else
            {
                gimbal_target.target_yaw_vel = -5.24f * DT7.get_right_x();
                gimbal_target.target_pitch1 = 0.0f;
                gimbal_target.target_pitch2_angle = HI12.GetAngle(1);
                gimbal_target.target_pitch2_vel = 3.14f * DT7.get_right_y();
            }

            // 在J1达到普通角度附近再对J2做限幅
            float vmc_theta1_actual = -MotorJ4340.getAngleDeg(1);
            float j1_target_abs = 103.0f + gimbal_target.target_pitch1;
            bool j1_ready_for_j2_limit = fabsf(vmc_theta1_actual - j1_target_abs) < 3.0f;

            if (j1_ready_for_j2_limit)
            {
                float pitch2_fb = HI12.GetAngle(1);

                if (pitch2_fb >= 33.0f && gimbal_target.target_pitch2_vel > 0.0f)
                {
                    gimbal_target.target_pitch2_vel = 0.0f;
                    gimbal_output.out_pitch2 = 0.0f;
                }
                else if (pitch2_fb <= -26.0f && gimbal_target.target_pitch2_vel < 0.0f)
                {
                    gimbal_target.target_pitch2_vel = 0.0f;
                    gimbal_output.out_pitch2 = 0.0f;
                }
            }
            break;
        }

        case TRANSFORM:
            gimbal_target.target_yaw_angle = -90.5f;       // yaw变形角度
            gimbal_target.target_pitch1 = 50.0f;    // 倒下角度103+50=153度
            gimbal_target.target_pitch2_angle = 0.0f;     // pitch2 保持世界坐标系下水平
            break;

        case VISION:
            gimbal_target.target_yaw_angle = vision_filter_yaw.filter(vision.getTarYaw());
            gimbal_target.target_pitch1 = 0.0f;
            gimbal_target.target_pitch2_angle = vision_filter_pitch.filter(vision.getTarPitch());
            break;

        default:
            // 与STOP一致
            gimbal_target.target_yaw_vel = 0.0f;
            gimbal_target.target_yaw_angle = MotorJ4310.getAngleDeg(2);
            gimbal_target.target_pitch1 = -MotorJ4340.getAngleDeg(1) - 103.0f;
            gimbal_target.target_pitch2_vel = 0.0f;
            gimbal_target.target_pitch2_angle = HI12.GetAngle(1);
            break;
    }

}

/**
 * @brief 发射机构期望
 * 
 */
void SetTarget_Launch()
{
    static int last_launch_state = -1;
    DT7.SetDeadzone(20.0f);
    switch(launch_fsm.Get_Now_State())
    {
        case LAUNCH_STOP:
            gimbal_target.target_surgewheel[0] = 0.0f;
            gimbal_target.target_surgewheel[1] = 0.0f;
            break;
        case LAUNCH_CEASEFIRE:
            gimbal_target.target_surgewheel[0] = 6000.0f;
            gimbal_target.target_surgewheel[1] = -6000.0f;
            break;
        case LAUNCH_ONLY:
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)
            {
                static bool last_mouse_left = false;
                if (alphabet[26] && !last_mouse_left)
                {
                    if(heat_control.GetNowFire() > 0)
                    {
                    }
                }
                last_mouse_left = alphabet[26];
                gimbal_target.target_surgewheel[0] = 6000.0f;
                gimbal_target.target_surgewheel[1] = -6000.0f;
            }
            else
            {
                static bool last_scroll_active = false;
                float scroll_val = DT7.get_scroll_();
                bool current_scroll_active = (fabsf(scroll_val) > 0.8f);
                if (last_scroll_active && !current_scroll_active)
                {
                    if(heat_control.GetNowFire() > 0)
                    {
                    }
                }
                last_scroll_active = current_scroll_active;
                gimbal_target.target_surgewheel[0] = 6000.0f;
                gimbal_target.target_surgewheel[1] = -6000.0f;
            }
            break;
        case LAUNCH_AUTO:
            gimbal_target.target_surgewheel[0] = 6000.0f;
            gimbal_target.target_surgewheel[1] = -6000.0f;
            break;
        case LAUNCH_JAM:
            gimbal_target.target_surgewheel[0] = 6000.0f;
            gimbal_target.target_surgewheel[1] = -6000.0f;
            break;
        default:
            gimbal_target.target_surgewheel[0] = 0.0f;
            gimbal_target.target_surgewheel[1] = 0.0f;
            break;
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
    yaw_adrc.Reset();
    yaw_transform_angle_pid.reset();
    yaw_transform_velocity_pid.reset();

    pitch2_adrc.Reset();


    gimbal_output.out_yaw = 0.0f;
    gimbal_output.out_pitch1 = 0.0f;
    gimbal_output.out_pitch2 = 0.0f;
}

void gimbal_manual()
{
    // Yaw 速控LADRC
    yaw_adrc.LADRC_1(gimbal_target.target_yaw_vel, HI12.GetGyroRad(2));

    // 模型前馈
    gimbal_yaw.MomentOfInertiaTuning(HI12.GetGyroRad(2), gimbal_target.target_yaw_vel);
    // 在有期望值才加模型前馈，避免零飘，gimbal_yaw.getFriction() 输出单位是Nm，需要转化成控制电流
    float model_comp = (fabsf(gimbal_target.target_yaw_vel) > 0.001f) ? gimbal_yaw.getTorque() : 0.0f;
    gimbal_output.out_yaw = yaw_adrc.GetU() + model_comp;
    gimbal_output.out_yaw = std::clamp(gimbal_output.out_yaw, -3.0f, 3.0f);
    // // UDE估计�?
    // static float last_total_out_yaw = 0.0f; // 由于因果律，需获取上一次的输出
    // float ude_input = last_total_out_yaw * (3.0f / 16384.0f); // 将控制电流转化为真是电流
    // ude_filter_input.filter(HI12.GetGyroRad(2));    // 获取X_dot，使用TD获取微分
    // ude_yaw.UDE_Update(ude_input, ude_filter_input.getDerivative());    // UDE计算
    // float ude_output = (ude_filter_out.filter(ude_yaw.getOutput())) * 16384.0f/3.0f;    // UDE输出转化为控制电�?
    // // 使用线性衰减来限制UDE的补偿，防止云台在急速移动时UDE补偿过大
    // float speed_error = fabsf(gimbal_target.target_yaw - HI12.GetGyroRPM(2)); // 计算期望速度与实际速度的误�?
    // float k_ude = 1.0f; // 系数
    // if(speed_error > 20.0f) 
    // {
    //     k_ude = 1.0f - (speed_error - 20.0f) / 80.0f; // 线性衰�?
    // }
    // k_ude = std::clamp(k_ude, 0.0f, 1.0f); // 保证系数�?�?之间
    // // 只采信安全平缓状态下的UDE补偿
    // float ude_output_safe = ude_output * k_ude;
    // // 小陀螺才开启UDE补偿
    // bool scroll = (gimbal_fsm.Get_Now_State() == MANUAL) && (launch_fsm.Get_Now_State() == LAUNCH_AUTO || launch_fsm.Get_Now_State() == LAUNCH_ONLY || launch_fsm.Get_Now_State() == LAUNCH_JAM);
    // if((!scroll && (DT7.get_scroll_() != 0.0f)) || alphabet[23])
    // {
    //     gimbal_output.out_yaw = yaw_pid.getOutput() + model_comp - ude_output_safe;
    // }
    // else
    // {
    //     gimbal_output.out_yaw = yaw_pid.getOutput() + model_comp;
    // }
    // // Yaw 输出，进行限�?
    // gimbal_output.out_yaw = std::clamp(gimbal_output.out_yaw, -16384.0f, 16384.0f);
    // last_total_out_yaw = gimbal_output.out_yaw; // 更新上一时刻的yaw输出，UDE�?


    // Yaw使用MIT和传内置Pid

    // === Pitch 控制 ===
    // J1: 使用VMC (末端 = J2 关节位置)
    float vmc_theta1 = -1.0f * MotorJ4340.getAngleDeg(1);           // 负号转换坐标
    float vmc_theta1_dot = -1.0f * MotorJ4340.getVelocityRads(1);   // 负号转换坐标

    // 更新重力前馈 
    gravity_forward_pitch1.GravityFeedforward(vmc_theta1);

    // 起立的J1滤波
    float smooth_pitch1 = vmc_pitch1_filter.filter(gimbal_target.target_pitch1);

    // 计算VMC
    doublePitch.Settheta(vmc_theta1, 0.0f);
    doublePitch.Settheta_dot(vmc_theta1_dot, 0.0f);
    doublePitch.VMC_Update(gimbal_target.target_pitch2_angle, smooth_pitch1);

    // J1 输出力矩 (轴向反转 + 重力补偿)
    gimbal_output.out_pitch1 = std::clamp(-doublePitch.GetT1() + gravity_forward_pitch1.getFeedforward(), -9.0f, 9.0f);

    // --- Pitch2 解耦控制 ---
    pitch2_adrc.LADRC_1(gimbal_target.target_pitch2_vel, HI12.GetGyroRad(0));
    gravity_forward_pitch2.GravityFeedforward(HI12.GetAngle(1));
    gimbal_output.out_pitch2 = std::clamp(pitch2_adrc.GetU() + gravity_forward_pitch2.getFeedforward(), -3.0f, 3.0f);
}

void gimbal_vision()
{

}

float a;
void gimbal_transform()
{
    // 1. 预处理目标值：让原始目标值“靠近”当前的反馈，处理过零点
    float raw_target = gimbal_target.target_yaw_angle;
    float current_fb = MotorJ4310.getAngleDeg(2);
    float selected_target = selectYawEquivalentTarget(raw_target, current_fb);
    float processed_target = getYawTargetProcessed(selected_target, current_fb);
    a = processed_target;
    // 2. TD 滤波：在处理后的“邻域角度”上进行平滑，防止云台猛甩
    static float filtered_target = 0;
    filtered_target = vmc_yaw_filter.filter(processed_target);

    // 3. 传入 PID 计算
    yaw_transform_angle_pid.UpDate(filtered_target, current_fb);
    //yaw_adrc_try.LADRC_1(yaw_transform_angle_pid.getOutput(), MotorJ4310.getVelocityRpm(2));
    yaw_transform_velocity_pid.UpDate(yaw_transform_angle_pid.getOutput(), MotorJ4310.getVelocityDegs(2));
    //gimbal_output.out_yaw = yaw_adrc_try.GetU();
    gimbal_output.out_yaw = yaw_transform_velocity_pid.getOutput() * 3.0f/16384.0f;
    

    // 判断 Yaw 轴是否已经接近目标位（误差小于 2 度）
    bool yaw_ready_for_pitch = fabsf(MotorJ4310.getAngleDeg(2) - processed_target) < 4.0f;

    // 获取当前机械实际姿态，用于在 Yaw 没对准前锁定位置
    float vmc_theta1 = -1.0f * MotorJ4340.getAngleDeg(1);
    float vmc_theta1_dot = -1.0f * MotorJ4340.getVelocityRads(1);
    float current_pitch1 = vmc_theta1 - 103.0f;
    float current_pitch2 = HI12.GetAngle(1);

    // 如果 Yaw 还没对准，Pitch 目标维持在“当前实际位置”，实现锁定效果
    // 如果 Yaw 已对准，Pitch 目标开始向 变形目标 移动
    float pitch1_target = yaw_ready_for_pitch ? gimbal_target.target_pitch1 : current_pitch1;
    float pitch2_target = yaw_ready_for_pitch ? gimbal_target.target_pitch2_angle : current_pitch2;

    // 通过 TD 滤波器实现 Pitch 的平滑运动
    float smooth_pitch1 = vmc_pitch1_filter.filter(pitch1_target);
    float smooth_pitch2 = vmc_pitch2_filter.filter(pitch2_target);

    // 更新重力前馈
    gravity_forward_pitch1.GravityFeedforward(vmc_theta1);

    // J1: VMC 控制
    doublePitch.Settheta(vmc_theta1, 0.0f);
    doublePitch.Settheta_dot(vmc_theta1_dot, 0.0f);
    doublePitch.VMC_Update(smooth_pitch2, smooth_pitch1);

    // J1 输出力矩限幅
    gimbal_output.out_pitch1 = std::clamp(-doublePitch.GetT1() + gravity_forward_pitch1.getFeedforward(), -9.0f, 9.0f);

    // J2: 角度速度PID闭环
    gravity_forward_pitch2.GravityFeedforward(HI12.GetAngle(1));
    pitch2_transform_angle_pid.UpDate(smooth_pitch2, HI12.GetAngle(1));
    pitch2_transform_velocity_pid.UpDate(pitch2_transform_angle_pid.getOutput(), HI12.GetGyro(0));
    
    // J2 输出力矩限幅
    gimbal_output.out_pitch2 = std::clamp(pitch2_transform_velocity_pid.getOutput()*3.0f/16384.0f + gravity_forward_pitch2.getFeedforward(), -3.0f, 3.0f);
}

/**
 * @brief 云台主控制循
 * 
 * @param left_sw 遥控器左开关状
 * @param right_sw 遥控器右开关状
 * @param is_online 设备在线状
 */
void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online) 
{   
    gimbal_fsm.StateUpdate(left_sw, right_sw, is_online, vision.getVisionFlag(), false);    // 最后一个传参要改成键鼠的键�?
    gimbal_fsm.TIM_Update();

    SetTarget_Gimbal();

    switch(gimbal_fsm.Get_Now_State()) 
    {
        case STOP:      // 停止模式
            gimbal_stop();
            break;
        case MANUAL:    // 普通模�?
            gimbal_manual();
            break;
        case VISION:    // 视觉模式
            gimbal_manual();    //用着�?
            //gimbal_vision();
            break;
        case TRANSFORM: // 变形模式
            gimbal_transform();
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
    // 速度环期望值置零，快速停�?
    // dial_pid.UpDate(gimbal_target.target_dial, MotorLK4005.getVelocityRpm(1));
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].UpDate(gimbal_target.target_surgewheel[i], Motor3508.getVelocityRpm(i + 2));
    }
    // 输出
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
    // launch_output.out_dial = dial_pid.getOutput();
}

/**
 * @brief 发射机构单发以及连发模式控制函数
 * 
 * 拨盘按角度控制，摩擦轮保持设定转�?
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

    // 拨盘角度�?
    // dial_angle_pid.UpDate(gimbal_target.target_dial, MotorLK4005.getAddAngleDeg(1));
    // dial_velocity_pid.UpDate(dial_angle_pid.getOutput(), MotorLK4005.getVelocityRpm(1));
    // launch_output.out_dial = dial_velocity_pid.getOutput();
}

/**
 * @brief 发射机构卡弹控制函数
 * 
 * 卡弹的时候输出反向力�?
 */
void launch_jam()
{
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].UpDate(gimbal_target.target_surgewheel[i], Motor3508.getVelocityRpm(i + 2));
    }
    // 输出
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
    // launch_output.out_dial = gimbal_target.target_dial;
}

/**
 * @brief 发射机构主控制循�?
 * 
 */
void main_loop_launch()
{
    // 根据发射状态机使能/失能拨盘电机
    Enum_Launch_States launch_state = launch_fsm.Get_Now_State();
    // if(launch_state != LAUNCH_STOP)
    // {
    //     // 非停止状态，使能拨盘
    //     if(!MotorLK4005.getIsenable(1))
    //     {
    //         MotorLK4005.setIsenable(1, true);
    //         MotorLK4005.setAllowAccumulate(1, true);
    //     }
    // }
    // else
    // {
    //     // 停止状态，失能拨盘
    //     if(MotorLK4005.getIsenable(1))
    //     {
    //         MotorLK4005.setIsenable(1, false);
    //     }
    // }

    SetTarget_Launch();

    // 基于 Action �?Mode 执行控制函数
    switch(launch_state)
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
 * @brief 卡弹检�?
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
    
    // 1. 位置误差超过 120 �?
    if (err > 120.0f * 36.0f)
    {
        uint32_t now = HAL_GetTick();
        
        // 初始�?
        if (last_check_time == 0)
        {
            last_check_time = now;
            last_angle = current_angle;
            return false;
        }
        
        // 2. �?300ms 检查一�?
        if (now - last_check_time > 300)
        {
            // 3. 如果�?300ms 内转动角度小�?120 �?-> 判定卡弹
            if (fabs(current_angle - last_angle) < 120.0f * 36.0f)
            {
                // 重置检测状态（因为即将进入 Jam 模式，Jam模式�?error 会变小）
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
        // 误差不大的时候，重置计时�?
        last_check_time = 0;
    }
    
    return false;
}


/* 控制任务部分 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 控制任务主函�?
 * 
 * @param argument 任务参数
 */
extern "C"{
void Control(void const * argument)
{
    // 初始化蜂鸣器管理器；初始�?310到失能；初始化状态机
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    fsm_init();

    static uint8_t control_tick = 0;
    for(;;)
    {
        // 更新蜂鸣器管理器，处理队列中的响铃请�?
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        // 1kHz 采样 (捕捉信号)
        
        // 热量控制
        float current[2] = {Motor3508.getCurrent(2), Motor3508.getCurrent(3)};
        float velocity[2] = {Motor3508.getVelocityRpm(2), Motor3508.getVelocityRpm(3)}; 
        heat_control.HeatControl(gimbal_target.target_surgewheel[0], current, velocity, Cboard.GetHeatLimit(), Cboard.GetHeatCool() , 0.001f, 13.0f);

        // 1000Hz�?00Hz
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
        
        osDelay(2);
    } 
}
}



