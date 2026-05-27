#include "MotorTask.hpp"
#include "ControlTask.hpp"

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<3> Motor3508(0x200, {1, 2, 3}, 0x200); // 1号ID是2006，避免ID冲突合并到3508中
BSP::Motor::DM::J4310<2> MotorJ4310(0x00, {6, 8}, {4, 7});

/* 板间通讯接收缓冲区 ----------------------------------------------------------------------------------*/
BSP::CANTransport::RxBuffer board_downlink_rx; // 收下板发来的 4 字节热量数据 (CAN ID 0x320, ChassisDownlink_RX)

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
        if (frame.id == 0x320) board_downlink_rx.feed(frame); // 板间通讯下行: 下板→云台
    });
}

/* 板间通讯接收 ------------------------------------------------------------------------------------------*/
/**
 * @brief 处理下板发来的裁判系统热量数据 (CAN ID 0x320)
 *
 * 数据包格式: ChassisDownlink_RX (4 字节)
 *   - shooter_barrel_cooling_value (uint16_t)
 *   - shooter_barrel_heat_limit    (uint16_t)
 */
static void process_board_downlink()
{
    if (board_downlink_rx.complete())
    {
        const auto *dl = reinterpret_cast<const ChassisDownlink_RX *>(board_downlink_rx.data());
        Cboard.updateTimestamp();
        Cboard.SetHeatData(dl);
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