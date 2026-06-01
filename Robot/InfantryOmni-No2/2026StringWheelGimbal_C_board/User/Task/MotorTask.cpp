#include "MotorTask.hpp"
#include "ControlTask.hpp"
float Motor4310_angle[2] = {0.0f, 0.0f};
float Motor4310_velocity[2] = {0.0f, 0.0f};

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<3> Motor3508(0x200, {1, 2, 3}, 0x200); // 1号ID是2006，避免ID冲突合并到3508中
BSP::Motor::DM::J4310<2> MotorJ4310(0x00, {6, 8}, {4, 7}); // 0: pitch, 1: yaw

/* 板间通讯接收 ----------------------------------------------------------------------------------------*/
BSP::CANTransport::RxBuffer board_downlink_rx;          // CAN ID 0x310, 接收底盘上行 BoardRx (28 字节)

/* CAN接收 ---------------------------------------------------------------------------------------------*/
/**
 * @brief CAN初始化函数
 * 
 * 初始化CAN总线并注册电机反馈数据解析回调函数
 */
void MotorInit(void)
{
    // 实例CAN
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    
    // 注册CAN接收回调函数
    can1.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        MotorJ4310.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        Motor3508.Parse(frame);
        if (frame.id == 0x310) board_downlink_rx.feed(frame); // 板间通讯: 底盘上行→云台
    });
}

/* 板间通讯接收 ------------------------------------------------------------------------------------------*/
/**
 * @brief 处理底盘上行数据 (CAN ID 0x310, BoardRx 28 字节)
 *
 * 包含: 里程计 + 速度 + 偏航 + 热量
 */
static void process_board_downlink()
{
    // 临界区: 与 CAN ISR 的 feed() 竞争 RxBuffer, 关中断保护 complete→copy→reset
    __disable_irq();
    bool done = board_downlink_rx.complete();
    if (done)
    {
        memcpy(&Cboard.board_rx_, board_downlink_rx.data(), sizeof(Cboard.board_rx_));
        board_downlink_rx.reset();
    }
    __enable_irq();

    if (done)
    {
        Cboard.updateTimestamp();
    }
}

/* 板间通讯发送 ------------------------------------------------------------------------------------------*/
/**
 * @brief 发送云台下行数据 (CAN ID 0x311, BoardTx 35 字节)
 *
 * 包含: 遥控器数据转发 + Motor4310 角度 + scroll 状态 + 底盘目标速度
 */
static void send_board_downlink()
{
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    memcpy(Cboard.board_tx_.rc_data, DT7Rx_buffer, sizeof(DT7Rx_buffer));
    Cboard.board_tx_.angle = MotorJ4310.getAngleRad(2);
    Cboard.board_tx_.scroll = (gimbal_fsm.Get_Now_State() == MANUAL)
        && (launch_fsm.Get_Now_State() == LAUNCH_AUTO
         || launch_fsm.Get_Now_State() == LAUNCH_ONLY
         || launch_fsm.Get_Now_State() == LAUNCH_JAM);
    Cboard.board_tx_.target_vx = navigation.getTargetVx();
    Cboard.board_tx_.target_vy = navigation.getTargetVy();
    Cboard.board_tx_.target_wz = navigation.getTargetWz();

    BSP::CANTransport::sendPacket(can2, kCanIdDownlink, &Cboard.board_tx_, sizeof(Cboard.board_tx_));
}

/* 电机发送控制帧 ----------------------------------------------------------------------------------------*/
/**
 * @brief 电机控制函数
 *
 * 根据控制系统的输出结果向各电机发送控制指令
 */
static void motor_control()
{
    // ──── 云台参数计算 ────────────────────────────────────────────────
    float pos_p = 0.0f, vel_p = 0.0f, kp_p = 0.0f, kd_p = 0.0f, torq_p = 0.0f;
    float pos_y = 0.0f, vel_y = 0.0f, kp_y = 0.0f, kd_y = 0.0f, torq_y = 0.0f;


    vel_p = gimbal_output.out_pitch_angle;
    kd_p = pitch_velocity_pid.getK(2);
    torq_p = gimbal_output.out_pitch;

    vel_y = -gimbal_output.out_yaw_angle;
    kd_y = yaw_velocity_pid.getK(2);
    torq_y = -gimbal_output.out_yaw;


    // ──── J4310 使能/失能 + 控制 (Pitch=索引1, Yaw=索引2) ──────────
    // 状态: 已使能→ctrl_Mit 控制帧; 未使能→On 唤醒帧; 失能→全零 ctrl_Mit 维持通讯
    // 每电机每周期仅 1 帧, 邮箱安全
    static bool pitch_enabled = false;
    static bool yaw_enabled   = false;

    // Pitch (索引1, 发往 CAN ID 4)
    if (gimbal_output.motor_pitch_enable)
    {
        if (MotorJ4310.GetErr(1) != 1)
            MotorJ4310.On(1, BSP::Motor::DM::MIT);
        else
        {
            pitch_enabled = true;
            MotorJ4310.ctrl_Mit(1, pos_p, vel_p, kp_p, kd_p, torq_p);
        }
    }
    else
    {
        if (pitch_enabled)
        {
            // Off 可能被 CAN 邮箱丢弃，持续发送直到电机反馈确认失能
            MotorJ4310.Off(1, BSP::Motor::DM::MIT);
            if (MotorJ4310.GetErr(1) != 1)
                pitch_enabled = false;
        }
        else
        {
            MotorJ4310.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    // Yaw (索引2, 发往 CAN ID 7)
    if (gimbal_output.motor_yaw_enable)
    {
        if (MotorJ4310.GetErr(2) != 1)
            MotorJ4310.On(2, BSP::Motor::DM::MIT);
        else
        {
            yaw_enabled = true;
            MotorJ4310.ctrl_Mit(2, pos_y, vel_y, kp_y, kd_y, torq_y);
        }
    }
    else
    {
        if (yaw_enabled)
        {
            // Off 可能被 CAN 邮箱丢弃，持续发送直到电机反馈确认失能
            MotorJ4310.Off(2, BSP::Motor::DM::MIT);
            if (MotorJ4310.GetErr(2) != 1)
                yaw_enabled = false;
        }
        else
        {
            MotorJ4310.ctrl_Mit(2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
    
    // 发射机构
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_dial), 1); 
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[0]), 2);
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[1]), 3);
    Motor3508.sendCAN();

    // 板间下行降频至 500Hz, 减少 CAN2 邮箱负载
    static uint8_t downlink_tick = 0;
    if (++downlink_tick % 5 == 0) 
    {
        send_board_downlink();
    }
}

/* 任务函数 --------------------------------------------------------------------------------------------*/
/**
 * @brief 电机控制任务函数
 * 
 * 任务主循环，负责初始化电机并持续执行电机控制
 * 
 * @param argument 任务参数指针
 */
extern "C"{
void Motor(void const * argument)
{
    MotorInit();    // 初始化电机和CAN总线
    for(;;)
    {
        process_board_downlink();   // 解析底盘上行 CAN 数据 (0x310)
        motor_control();            // 执行电机控制 (含下行 CAN 发送)
        for(int i = 0; i < 2; ++i)
        {
            Motor4310_angle[0] = HI12.GetAngle(1) * 3.14159f / 180.0f; // 弧度rad
            Motor4310_angle[1] = MotorJ4310.getAngleRad(i + 1);
            Motor4310_velocity[i] = MotorJ4310.getVelocityRads(i + 1);
        }
        osDelay(1);                 // 延时1ms，控制频率1000Hz
    }
}

}