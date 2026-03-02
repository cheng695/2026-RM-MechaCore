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

/* 斜坡规划 -------------------------------------------------------------------------------------------------*/
Alg::Utility::SlopePlanning slopePlanning_pitch1(0.1f, 0.1f);
Alg::Utility::SlopePlanning slopePlanning_pitch2(0.1f, 0.1f);

/* 滤波器 ---------------------------------------------------------------------------------------------------*/
TDFilter vision_filter_pitch(30.0f, 0.005f);    // 视觉pitch滤波器
TDFilter vision_filter_yaw(40.0f, 0.005f);      // 视觉yaw滤波器

// 新增 VMC 目标平滑滤波器 (速度因子R决定运动快慢, 步长h=0.001s 因为你的控制周期是1ms)
TDFilter vmc_pitch1_filter(10.0f, 0.001f); // 调整R以控制收脖子的优雅程度，R越小越慢
TDFilter vmc_pitch3_filter(6.0f, 0.001f);
TDFilter vmc_pitch2_filter(200.0f, 0.001f); // J2稍微快一点

/* 前馈 -----------------------------------------------------------------------------------------------------*/
Alg::Feedforward::Friction friction_forward(200.0f);        // 摩擦力前馈（普通模式 Yaw）
Alg::Feedforward::Gravity gravity_forward(-0.85f, 115.0f);  // 重力前馈（所有模式 Pitch）
Alg::Feedforward::Acceleration acc_forward(0.5f, 0.005f);   // 加速度前馈（普通模式 Yaw）
Alg::Feedforward::Velocity velocity_forward(0.5f, 0.005f);  // 速度前馈（视觉模式 Yaw）

Alg::Feedforward::Gravity gravity_forward_pitch1(0.4f, 0.0f);
Alg::Feedforward::Gravity gravity_forward_pitch2(0.0f, 0.0f);

/* 控制器 ---------------------------------------------------------------------------------------------------*/
ALG::PID::PID yaw_pid(400.0f, 10.0f, 0.0f, 16384.0f, 30.0f, 3.0f);  // yaw轴速控pid（普通模式）
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

ALG::PID::PID dm4310(60.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
ALG::PID::PID dm4340(150.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f);
ALG::VMC::VMC doublePitch(0.16f, 0.1f, 400.0f, 500.0f, 30.0f, 35.0f);
ALG::VMC::VMC doublePitch2(0.16f, 0.1f, 20000.0f, 500.0f, 100.0f, 35.0f);

ALG::PID::PID lk4005(0.0f, 0.0f, 0.0f, 2048.0f, 0.0f, 0.0f);

/* 热量控制 --------------------------------------------------------------------------------------------------*/
APP::Heat_Control_Private heat_control(100, 50, 30.0f, 1.0f, 0.001f);

/* 期望值与输出 ----------------------------------------------------------------------------------------------*/
ControlTask gimbal_target;      // 云台与发射机构期望值

Output_gimbal gimbal_output;    // 云台输出
Output_launch launch_output;    // 发射机构输出

Enum_Gimbal_States_TEXT State_gimbal_text;
uint32_t transform_start_time = 0;

/**
 * @brief 云台与发射机构控制逻辑
 */
/* 公共部分 ---------------------------------------------------------------------------------------------------*/

void statemachine(uint8_t left_sw, uint8_t right_sw)
{
    static Enum_Gimbal_States_TEXT last_state = GIMBAL_STOP;
    Enum_Gimbal_States_TEXT next_state = State_gimbal_text;

    if(left_sw == 2 && right_sw == 2)
    {
        next_state = GIMBAL_STOP;
    }
    else if(left_sw == 3 && right_sw == 2)
    {
        next_state = GIMBAL_MANUAL;
    }
    else if(left_sw == 3 && right_sw == 3)
    {
        next_state = GIMBAL_TRANSFORM;
    }

    if(last_state != GIMBAL_TRANSFORM && next_state == GIMBAL_TRANSFORM)
    {
        transform_start_time = HAL_GetTick();
    }

    if(last_state == GIMBAL_TRANSFORM && next_state == GIMBAL_MANUAL)
    {
        gimbal_target.target_pitch2 = 0.0f;
    }

    State_gimbal_text = next_state;
    last_state = next_state;
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

    switch (State_gimbal_text)
    {
    case GIMBAL_STOP:
        gimbal_target.target_pitch1 = 68.0f;
        gimbal_target.target_pitch2 -= DT7.get_right_y();
        gimbal_target.target_pitch2 = std::clamp(gimbal_target.target_pitch2, -30.0f, 30.0f);  // 角度deg
        gimbal_target.target_dial = 0.0f;
        break;
    case GIMBAL_MANUAL:
        gimbal_target.target_pitch1 = 0.0f;
        gimbal_target.target_pitch2 -= 0.3f*DT7.get_right_y();
        gimbal_target.target_pitch2 = std::clamp(gimbal_target.target_pitch2, -30.0f, 30.0f); 
        gimbal_target.target_dial = 316.0f * DT7.get_scroll_();
        break;
    case GIMBAL_TRANSFORM:
        gimbal_target.target_pitch1 = 68.0f;
        gimbal_target.target_pitch2 = 0.0f;
        break;
    }
}

/* 云台部分 ---------------------------------------------------------------------------------------------------*/

/**
 * @brief 云台停止模式控制函数
 * 
 * 关闭电机使能并重置控制器参数
 */
void gimbal_stop()
{
    static uint8_t send_seq = 0;
    send_seq++;

        if(MotorLK4005.getIsenable())
        {
            MotorLK4005.Off(1, 1);
            MotorJ4310.setIsenable(false);
        }

    // if (send_seq % 2 == 0) 
    // {
    //     if(MotorJ4310.getIsenable())
    //     {
    //         MotorJ4310.Off(1, BSP::Motor::DM::MIT);
    //         MotorJ4310.ClearErr(1, BSP::Motor::DM::MIT);
    //         if(MotorJ4310.getError(1)==0)
    //         {
    //             MotorJ4310.setIsenable(false);
    //         }
    //     }
    // }
    // else 
    // {
    //     if(MotorJ4340.getIsenable())
    //     {
    //         MotorJ4340.Off(1, BSP::Motor::DM::MIT);
    //         MotorJ4340.ClearErr(1, BSP::Motor::DM::MIT);
    //         if(MotorJ4340.getError(1)==0)
    //         {
    //             MotorJ4340.setIsenable(false);
    //         }
    //     }
    // }
    // // 失能4310


    // // 重置控制器
    // yaw_pid.reset();
    // yaw_angle_pid.reset();
    // yaw_velocity_pid.reset();
    // // 输出置零
    // gimbal_output.out_yaw = 0.0f;
    // gimbal_output.out_pitch = 0.0f;

    dm4310.reset();
    dm4340.reset();
    lk4005.reset();
    launch_output.out_dial = 0.0f;
    gimbal_output.out_pitch1 = 0.0f;
    gimbal_output.out_pitch2 = 0.0f;
}


void gimbal_text()
{
    static uint8_t send_seq = 0;
    send_seq++;

    if (send_seq % 2 == 0) 
    {
        if(!MotorJ4310.getIsenable())
        {
            MotorJ4310.On(1, BSP::Motor::DM::MIT);
            if(MotorJ4310.getError(1)==1)
            {
                MotorJ4310.setIsenable(true);
            }
        }
    }
    else 
    {
        if(!MotorJ4340.getIsenable())
        {
            MotorJ4340.On(1, BSP::Motor::DM::MIT);
            if(MotorJ4340.getError(1)==1)
            {
                MotorJ4340.setIsenable(true);
            }
        }
    }

    doublePitch.Settheta(MotorJ4340.getAngleDeg(1), MotorJ4310.getAngleDeg(1));
    doublePitch.Settheta_dot(MotorJ4340.getVelocityRads(1), MotorJ4310.getVelocityRads(1));
    
    float smooth_pitch = vmc_pitch1_filter.filter(gimbal_target.target_pitch1);
    float smooth_pitch1 = vmc_pitch3_filter.filter(gimbal_target.target_pitch1);

    doublePitch.VMC_Update(gimbal_target.target_pitch2, smooth_pitch1);

    gravity_forward_pitch1.GravityFeedforward(MotorJ4340.getAngleDeg(1));
    gimbal_output.out_pitch1 = std::clamp(doublePitch.GetT1() + gravity_forward_pitch1.getFeedforward(), -9.0f, 9.0f);
    gimbal_output.out_pitch2 = std::clamp(doublePitch.GetT2(), -3.0f, 3.0f);
    //dm4310.UpDate()
}

void gimbal_transform()
{
    static uint8_t send_seq = 0;
    send_seq++;

    if (send_seq % 2 == 0) 
    {
        if(!MotorJ4310.getIsenable())
        {
            MotorJ4310.On(1, BSP::Motor::DM::MIT);
            if(MotorJ4310.getError(1)==1)
            {
                MotorJ4310.setIsenable(true);
            }
        }
    }
    else 
    {
        if(!MotorJ4340.getIsenable())
        {
            MotorJ4340.On(1, BSP::Motor::DM::MIT);
            if(MotorJ4340.getError(1)==1)
            {
                MotorJ4340.setIsenable(true);
            }
        }
    }

    // 平滑目标角度，防止瞬间力矩突变打齿
    float smooth_pitch = vmc_pitch3_filter.filter(gimbal_target.target_pitch1);
    float smooth_pitch1 = vmc_pitch1_filter.filter(gimbal_target.target_pitch1);
    float smooth_pitch2 = vmc_pitch2_filter.filter(gimbal_target.target_pitch2);

    extern uint32_t transform_start_time;
    if (HAL_GetTick() - transform_start_time > 1000)
    {
        // 1秒后（变形完成），切入高刚度
        doublePitch2.Settheta(MotorJ4340.getAngleDeg(1), MotorJ4310.getAngleDeg(1));
        doublePitch2.Settheta_dot(MotorJ4340.getVelocityRads(1), MotorJ4310.getVelocityRads(1));
        doublePitch2.VMC_Update(smooth_pitch2, smooth_pitch1);

        gravity_forward_pitch1.GravityFeedforward(MotorJ4340.getAngleDeg(1));
        gimbal_output.out_pitch1 = std::clamp(doublePitch2.GetT1() + gravity_forward_pitch1.getFeedforward(), -9.0f, 9.0f);
        gimbal_output.out_pitch2 = std::clamp(doublePitch2.GetT2(), -3.0f, 3.0f);
    }
    else
    {
        // 1秒内（变形途中），使用柔和低刚度
        doublePitch.Settheta(MotorJ4340.getAngleDeg(1), MotorJ4310.getAngleDeg(1));
        doublePitch.Settheta_dot(MotorJ4340.getVelocityRads(1), MotorJ4310.getVelocityRads(1));
        doublePitch.VMC_Update(smooth_pitch2, smooth_pitch1);

        gravity_forward_pitch1.GravityFeedforward(MotorJ4340.getAngleDeg(1));
        gimbal_output.out_pitch1 = std::clamp(doublePitch.GetT1() + gravity_forward_pitch1.getFeedforward(), -9.0f, 9.0f);
        gimbal_output.out_pitch2 = std::clamp(doublePitch.GetT2(), -3.0f, 3.0f);
    }
}

void launchtext()
{
        if(!MotorLK4005.getIsenable())
        {
            MotorLK4005.On(1, BSP::Motor::DM::MIT);
            MotorJ4310.setIsenable(true);
        }

    lk4005.UpDate(gimbal_target.target_dial, MotorLK4005.getVelocityRpm(1));
    launch_output.out_dial = lk4005.getOutput();
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
    statemachine(left_sw, right_sw);
    SetTarget();

    if(State_gimbal_text == GIMBAL_STOP)
    {
        gimbal_stop();
    }
    else if(State_gimbal_text == GIMBAL_MANUAL)
    {
        gimbal_text();
    }
    else if(State_gimbal_text == GIMBAL_TRANSFORM)
    {
        gimbal_transform();
    }
}

bool check_online()
{
    return true;
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
    MotorJ4310.Off(0x01, BSP::Motor::DM::MIT);

    for(;;)
    {
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online());
        osDelay(1);
    } 
}
}

