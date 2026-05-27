#include "MotorTask.hpp"
#include "ControlTask.hpp"

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
    if (board_downlink_rx.complete())
    {
        memcpy(&Cboard.board_rx_, board_downlink_rx.data(), sizeof(Cboard.board_rx_));
        Cboard.updateTimestamp();
        board_downlink_rx.reset();
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
    // 云台 J4310 (Pitch=4, Yaw=7), 应答式通讯, 必须发包才能收到反馈
    float pos_p = 0.0f, vel_p = 0.0f, kp_p = 0.0f, kd_p = 0.0f, torq_p = 0.0f;
    float pos_y = 0.0f, vel_y = 0.0f, kp_y = 0.0f, kd_y = 0.0f, torq_y = 0.0f;

    auto state = gimbal_fsm.Get_Now_State();
    if (state == MANUAL)
    {
        // 速度闭环 → 力矩输出
        torq_p = gimbal_output.out_pitch;
        torq_y = gimbal_output.out_yaw;
    }
    else if (state == VISION)
    {
        // 角度闭环 PID → 速度期望, 力矩项用于前馈补偿
        kd_p = 0.0f;
        vel_p = 0.0f;
        torq_p = gimbal_output.out_pitch;
        
        kd_y = 0.0f;
        vel_y = 0.0f;
        torq_y = gimbal_output.out_yaw;
    }
    // else: 全零, 仅维持通讯

    MotorJ4310.ctrl_Mit(4, pos_p, vel_p, kp_p, kd_p, torq_p);
    MotorJ4310.ctrl_Mit(7, pos_y, vel_y, kp_y, kd_y, torq_y);
    
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
        osDelay(1);                 // 延时1ms，控制频率1000Hz
    }
}

}