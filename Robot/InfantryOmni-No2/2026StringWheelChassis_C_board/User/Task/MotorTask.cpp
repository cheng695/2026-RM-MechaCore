#include "MotorTask.hpp"
#include "Lk_motor.hpp"
#include "../../User/core/BSP/Common/CANTransport/CANTransport.hpp"

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);
BSP::Motor::LK::LK4005<4> Motor4005(0x140, {1, 2, 3, 4}, {1, 2, 3, 4});

/* 板间CAN通讯接收缓冲区 ----------------------------------------------------------------------------------*/
BSP::CANTransport::RxBuffer board_downlink_rx;  // 收上板下发的 23 字节数据 (CAN ID 0x300)

/* CAN2 上行发送缓存 --------------------------------------------------------------------------------------*/
static ChassisFeedbackCan uplink_fb{}; // C板→MiniPC 反馈数据

/*CAN接收 ---------------------------------------------------------------------------------------------*/
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
        Motor3508.Parse(frame);
        Motor4005.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        supercap.Parse(frame);                              // 超电: CAN ID 0x777
        if (frame.id == 0x300) board_downlink_rx.feed(frame); // 板间通讯下行: CAN ID 0x300
    });
}


/**
 * @brief 处理上板下发的板间数据 (CAN ID 0x300)
 *
 * 拼包完成后写入 BoardRx[23]，复用原有 UART6 解析逻辑。
 */
static void process_board_downlink()
{
    if (board_downlink_rx.complete())
    {
        const uint8_t *data = board_downlink_rx.data();
        memcpy(BoardRx, data, 23);
        Cboard.updateTimestamp();
        DT7.parseData(BoardRx);
        Cboard.SetYawAngle(BoardRx + 18);
        Cboard.SetScroll(BoardRx + 22);
        board_downlink_rx.reset();
    }
}

/**
 * @brief 板间通讯 CAN 发送 (C板 → MiniPC)
 *
 * 将 ChassisFeedbackCan 通过 CAN2 分包发送。
 * CAN ID: 0x310, 30 字节拆为 5 帧。
 */
static void BoardCommunicationTX()
{
    auto &can2 = HAL::CAN::get_can_bus_instance().get_can2();

    uplink_fb.vx = static_cast<float>(string_fk.GetChassisVx());
    uplink_fb.vy = static_cast<float>(string_fk.GetChassisVy());
    uplink_fb.wz = static_cast<float>(string_fk.GetChassisVw());
    uplink_fb.stamp_ms = HAL_GetTick();

    const auto *p = reinterpret_cast<const uint8_t *>(&uplink_fb);
    uint32_t sum = 0;
    for (size_t i = 0; i < offsetof(ChassisFeedbackCan, checksum); ++i)
        sum += p[i];
    uplink_fb.checksum = static_cast<uint16_t>(sum & 0xFFFF);

    BSP::CANTransport::sendPacket(can2, 0x310, &uplink_fb, sizeof(uplink_fb));
}

static void motor_control_logic(uint32_t tick)
{
    process_board_downlink();

    // 200Hz for Motors & Uplink (Every 5ms)
    if (tick % 5 == 0)
    {
        for (int i = 0; i < 4; i++)
        {
            Motor3508.setCAN(static_cast<int16_t>(chassis_output.out_wheel[i]), i + 1);
            Motor4005.ctrl_Torque(i + 1, static_cast<int16_t>(chassis_output.out_string[i]));
        }
        Motor3508.sendCAN();
        BoardCommunicationTX();
    }

    // 500Hz for Supercap (Every 2ms)
    if (tick % 2 == 0)
    {
        supercap.sendCAN();
    }
}                               

extern "C"{
void Motor(void const * argument)
{
    MotorInit();
    static uint32_t loop_count = 0;
    for(;;)
    {
        loop_count++;
        motor_control_logic(loop_count);
        osDelay(1);
    } 
}

}

