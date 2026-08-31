#include "GimbalController.hpp"
#include "Feedforward.hpp"
#include "Task/ControlTask.hpp"
#include "Module/Gimbal/GimbalInputBuilder.hpp"
#include "Module/Gimbal/GimbalPlanner.hpp"
#include "Module/Launch/LaunchInputBuilder.hpp"
#include "Module/Launch/LaunchPlanner.hpp"
#include "VMC/VMC.hpp"
#include <algorithm>

/* 滤波器 ---------------------------------------------------------------------------------------------------*/
TDFilter ude_filter_input(40.0f, 0.001f);   // UDE滤波器, 用于获取差分信号
TDFilter ude_filter_out(20.0f, 0.001f);     // UDE滤波器, 用于最终输出滤波

/* 前馈 -----------------------------------------------------------------------------------------------------*/
Alg::Feedforward::Gravity gravity_forward_J2(0.25f, -90.0f);  // 重力前馈（所有模式 Pitch）
Alg::Feedforward::Gravity gravity_forward_J1(0.0f, 0.0f);     // 重力前馈（所有模式 Pitch）

Alg::Feedforward::GimbalFullCompensation gimbal_yaw(0.005f, 0.001f, 0.005369f, 0.032416f);  // k_J初始为0，控制周期1ms
Alg::Feedforward::UDE ude_yaw(-0.39f, 6.6f);  // UDE前馈

/* 控制器 ---------------------------------------------------------------------------------------------------*/
ALG::VMC::VMC pitch_J1_vmc(0.14f, 0.0f, 2000.0f, 900.0f, 100.0f, 35.0f);

ALG::PID::PID yaw_angle_pid(20.0f, 0.0f, 0.0f, 12.5664f, 0.0f, 0.0f);    // yaw轴角速度pid（视觉模式）
ALG::PID::PID yaw_velocity_pid(0.0f, 0.0f, 1.0f, 16384.0f, 0.0f, 0.0f);  // yaw轴角速度pid（内置速度给Kd）

ALG::PID::PID pitch_angle_pid(20.0f, 0.0f, 0.0f, 12.5664f, 0.0f, 0.0f);        // pitch轴角速度pid（视觉模式）
ALG::PID::PID pitch_velocity_pid(0.0f, 0.0f, 1.0f, 16384.0f, 0.0f, 0.0f); // pitch轴角速度pid（内置速度给Kd）


/* 规划层统一生成云台和发射机构目标。 */
void SetTarget()
{
    DT7.SetDeadzone(20.0f);

    GimbalInput gimbal_input = BuildGimbalInput(operator_input);
    gimbal_input.current_yaw = HI12.GetAddYaw() * 3.14159f / 180.0f;
    gimbal_input.current_pitch = HI12.GetAngle(1) * 3.14159f / 180.0f;
    gimbal_input.mouse_x = DT7.get_mouseX();
    gimbal_input.mouse_y = DT7.get_mouseY();
    gimbal_input.right_x = DT7.get_right_x();
    gimbal_input.right_y = DT7.get_right_y();
    gimbal_input.vision_yaw = vision.getTarYaw();
    gimbal_input.vision_pitch = vision.getTarPitch();

    const GimbalTarget planned_gimbal = PlanGimbalTarget(
        gimbal_fsm.Get_Now_State(), gimbal_fsm.Get_Transition(), gimbal_input,
        {gimbal_target.target_yaw, gimbal_target.target_J1, gimbal_target.target_J2});
    gimbal_target.target_yaw = planned_gimbal.target_yaw;
    gimbal_target.target_pitch = planned_gimbal.target_J2;
    gimbal_target.target_J1 = planned_gimbal.target_J1;

    LaunchInput launch_input = BuildLaunchInput(operator_input,
                                                check_online(),
                                                launch_fsm.Get_Now_State() == LAUNCH_JAM);
    launch_input.current_dial_angle = Motor3508.getAddAngleDeg(1);
    launch_input.fire_rate = heat_control.GetNowFire();
    launch_input.scroll = DT7.get_scroll_();

    const LaunchTarget planned_launch = PlanLaunchTarget(
        launch_fsm.Get_Now_State(), launch_input,
        {gimbal_target.target_dial,
         {gimbal_target.target_surgewheel[0], gimbal_target.target_surgewheel[1]}});
    gimbal_target.target_dial = planned_launch.target_dial;
    gimbal_target.target_surgewheel[0] = planned_launch.target_surgewheel[0];
    gimbal_target.target_surgewheel[1] = planned_launch.target_surgewheel[1];
}

void gimbal_stop()
{
    // 重置控制器
    pitch_angle_pid.reset();
    pitch_velocity_pid.reset();
    yaw_angle_pid.reset();
    yaw_velocity_pid.reset();
    // 输出置零
    gimbal_output.out_yaw_torque = 0.0f;
    gimbal_output.out_J2_torque = 0.0f;
}

/**
 * @brief 云台普通模式控制函数
 * 
 * 实现手动遥控控制云台运动
 */
void gimbal_manual()
{
    // Yaw 角度PID
    yaw_angle_pid.UpDate(gimbal_target.target_yaw, HI12.GetAddYaw() * 3.14159f / 180.0f); // 弧度rad
    // Yaw 输出，进行限幅
    gimbal_output.out_yaw_angle = yaw_angle_pid.getOutput();
    // Yaw 前馈
    gimbal_output.out_yaw_torque = 0.0f;
    gimbal_output.out_yaw_torque = std::clamp(gimbal_output.out_yaw_torque, -3.0f, 3.0f);
    
    // Pitch 角度PID
    pitch_angle_pid.UpDate(gimbal_target.target_pitch, HI12.GetAngle(1) * 3.14159f / 180.0f);
    gimbal_output.out_pitch_angle = pitch_angle_pid.getOutput();
    // Pitch 重力补偿； Pitch控制用内置pid
    gravity_forward.GravityFeedforward(HI12.GetPitch_180());
    float gravity_comp = gravity_forward.getFeedforward();
    // Pitch 输出（放入MIT的扭矩项）
    gimbal_output.out_J2_torque = gravity_comp;
    gimbal_output.out_J2_torque = std::clamp(gimbal_output.out_J2_torque, -3.0f, 3.0f);
}

/**
 * @brief 云台视觉模式控制函数
 * 
 * 实现视觉引导的云台控制
 */
void gimbal_vision()
{
    // 过零处理，保证走最短路径 (适用于[-π, π]的突变)
    float current_yaw = HI12.GetAngle(2) * 3.14159f / 180.0f;
    float err_yaw = gimbal_target.target_yaw - current_yaw;

    while (err_yaw > 3.14159f)  err_yaw -= 6.28318f;
    while (err_yaw < -3.14159f) err_yaw += 6.28318f;

    // Yaw 角度PID
    yaw_angle_pid.UpDate(current_yaw + err_yaw, current_yaw);
    gimbal_output.out_yaw_angle = yaw_angle_pid.getOutput(); 

    // Yaw 前馈
    gimbal_output.out_yaw_torque = 0.0f;

    // Pitch 角速度PID
    pitch_angle_pid.UpDate(gimbal_target.target_pitch, HI12.GetAngle(1) * 3.14159f / 180.0f);
    gimbal_output.out_pitch_angle = pitch_angle_pid.getOutput();

    // Pitch 重力补偿;
    gravity_forward.GravityFeedforward(HI12.GetPitch_180());
    float gravity_comp = gravity_forward.getFeedforward();
    // Pitch 输出（放入MIT的扭矩项）
    gimbal_output.out_J2_torque = gravity_comp;
}

/**
 * @brief 云台导航模式控制函数
 *
 * Yaw 角度闭环匀速旋转, Pitch 正弦波角度闭环扫描
 */
void gimbal_navigation()
{
    yaw_angle_pid.reset();
    yaw_velocity_pid.reset();
    pitch_angle_pid.reset();
    pitch_velocity_pid.reset();

    gimbal_output.out_yaw_torque = 0.0f;
    gimbal_output.out_J2_torque = 0.0f;
    // gimbal_output.motor_pitch_enable = true; // 通知 MotorTask 使能 4310 Pitch
    // gimbal_output.motor_yaw_enable   = true; // 通知 MotorTask 使能 4310 Yaw

    // // Yaw 角度PID 跟踪持续递增的目标
    // yaw_angle_pid.UpDate(gimbal_target.target_yaw, HI12.GetAddYaw() * 3.14159f / 180.0f);
    // gimbal_output.out_yaw_angle = yaw_angle_pid.getOutput();
    // gimbal_output.out_yaw = 0.0f;
    // gimbal_output.out_yaw = std::clamp(gimbal_output.out_yaw, -3.0f, 3.0f);

    // // Pitch 角度PID 跟踪正弦波目标
    // pitch_angle_pid.UpDate(gimbal_target.target_pitch, HI12.GetAngle(1) * 3.14159f / 180.0f);
    // gimbal_output.out_pitch_angle = pitch_angle_pid.getOutput();
    // gravity_forward.GravityFeedforward(HI12.GetPitch_180());
    // float gravity_comp = gravity_forward.getFeedforward();
    // gimbal_output.out_pitch = gravity_comp;
    // gimbal_output.out_pitch = std::clamp(gimbal_output.out_pitch, -3.0f, 3.0f);
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
        // 跨越模式时清空陈旧的积分（I）和微分（D）误差，防止参数跳变导致”疯头”
        yaw_angle_pid.reset();
        yaw_velocity_pid.reset();
        pitch_angle_pid.reset();
        pitch_velocity_pid.reset();

        // 状态切换时将目标值置为当前陀螺仪角度，防止跳变
        gimbal_target.target_yaw   = HI12.GetAddYaw() * 3.14159f / 180.0f;
        gimbal_target.target_pitch = HI12.GetAngle(1) * 3.14159f / 180.0f;

        last_gimbal_state = gimbal_fsm.Get_Now_State();
    }

    SetTarget();

    RunGimbalController();
}



void RunGimbalController()
{
    main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online());
}


