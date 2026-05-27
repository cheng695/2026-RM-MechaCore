#include "MotorTask.hpp"
#include "ControlTask.hpp"

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<3> Motor3508(0x200, {1, 2, 3}, 0x200); // 1号ID是2006，避免ID冲突合并到3508中
BSP::Motor::DM::J4310<2> MotorJ4310(0x00, {6, 8}, {4, 7});

/* 板间通讯接收 ----------------------------------------------------------------------------------------*/
BSP::CANTransport::RxBuffer board_downlink_rx;          // CAN ID 0x310, 接收底盘 UplinkPacket_TX (34 字节)
UplinkPacket_RX           gimbal_uplink{};              // 底盘上行数据副本 (导航 + 裁判系统)

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
 * @brief 处理底盘上行数据 (CAN ID 0x310, UplinkPacket_RX 34 字节)
 *
 * 包含: NavigationData_RX (里程计 + 速度 + 偏航) + RefereeData_RX (热量)
 */
static void process_board_downlink()
{
    if (board_downlink_rx.complete())
    {
        memcpy(&gimbal_uplink, board_downlink_rx.data(), sizeof(gimbal_uplink));

        // 导航数据 checksum 校验 (前 28 字节累加和低 16 位)
        const auto *nav_p = reinterpret_cast<const uint8_t *>(&gimbal_uplink.nav);
        uint32_t nav_sum = 0;
        for (size_t i = 0; i < offsetof(NavigationData_RX, checksum); ++i)
            nav_sum += nav_p[i];

        if (static_cast<uint16_t>(nav_sum & 0xFFFF) == gimbal_uplink.nav.checksum)
        {
            Cboard.updateTimestamp();
            Cboard.SetNavData(&gimbal_uplink.nav);
            Cboard.SetHeatData(&gimbal_uplink.referee);
        }
        board_downlink_rx.reset();
    }
}

/* 电机发送控制帧 ----------------------------------------------------------------------------------------*/
/**
 * @brief 电机控制函数
 *
 * 根据控制系统的输出结果向各电机发送控制指令
 */
static void motor_control()
{
    // 云台
    if(gimbal_fsm.Get_Now_State() == MANUAL)
    {
        // Pitch
        MotorJ4310.ctrl_Mit(4, 
                           0.0f, 
                           0.0f, 
                           0.0f, 
                           0.0f, 
                           gimbal_output.out_pitch);
        // Yaw
        MotorJ4310.ctrl_Mit(7, 
                           0.0f, 
                           0.0f, 
                           0.0f, 
                           0.0f, 
                           gimbal_output.out_yaw);
    }
    else if(gimbal_fsm.Get_Now_State() == VISION)
    {
        // Pitch
        MotorJ4310.ctrl_Mit(4, 
                           0.0f, 
                           0.0f, 
                           0.0f, 
                           0.0f, 
                           gimbal_output.out_pitch);
        // Yaw
        MotorJ4310.ctrl_Mit(7, 
                           0.0f, 
                           0.0f, 
                           0.0f, 
                           0.0f, 
                           gimbal_output.out_yaw);
    }
    else
    {
        MotorJ4310.ctrl_Mit(1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    }
    
    // 发射机构
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_dial), 1); 
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[0]), 2);
    Motor3508.setCAN(static_cast<int16_t>(launch_output.out_surgewheel[1]), 3);
    Motor3508.sendCAN();
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
        process_board_downlink();   // 处理下板CAN数据
        motor_control();            // 执行电机控制
        osDelay(1);       // 延时1ms，控制频率1000Hz
    }
}

}