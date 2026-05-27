#include "MotorTask.hpp"
#include "Lk_motor.hpp"
#include "../../User/core/BSP/Common/CANTransport/CANTransport.hpp"

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);
BSP::Motor::LK::LK4005<4> Motor4005(0x140, {1, 2, 3, 4}, {1, 2, 3, 4});

/* 板间CAN通讯接收缓冲区 ----------------------------------------------------------------------------------*/
BSP::CANTransport::RxBuffer board_downlink_rx;  // 收上板下发的 23 字节数据 (CAN ID 0x300)

/* CAN2 上行发送缓存 --------------------------------------------------------------------------------------*/
static UplinkPacket_TX uplink_pkt{}; // C板→MiniPC 合包 (导航 + 裁判系统)
static NavigationData_TX &uplink_nav = uplink_pkt.nav;
static RefereeData_TX   &uplink_ref = uplink_pkt.referee;

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

    // 导航
    uplink_nav.vx = static_cast<float>(string_fk.GetChassisVx());
    uplink_nav.vy = static_cast<float>(string_fk.GetChassisVy());
    uplink_nav.wz = static_cast<float>(string_fk.GetChassisVw());
    uplink_nav.stamp_ms = HAL_GetTick();

    // 导航协议 checksum (前 28 字节)
    const auto *nav_p = reinterpret_cast<const uint8_t *>(&uplink_nav);
    uint32_t nav_sum = 0;
    for (size_t i = 0; i < offsetof(NavigationData_TX, checksum); ++i)
        nav_sum += nav_p[i];
    uplink_nav.checksum = static_cast<uint16_t>(nav_sum & 0xFFFF);

    // 裁判系统
    uplink_ref.shooter_barrel_cooling_value = ext_power_heat_data_0x0201.shooter_barrel_cooling_value;
    uplink_ref.shooter_barrel_heat_limit    = ext_power_heat_data_0x0201.shooter_barrel_heat_limit;

    BSP::CANTransport::sendPacket(can2, 0x310, &uplink_pkt, sizeof(uplink_pkt));
}

static void motor_control_logic(uint32_t tick)
{
    process_board_downlink();

    // 500Hz 基频, 3508/4005 交替发送各 250Hz (Every 2ms)
    // CAN1 每周期仅 1 帧: 3508 (0x200 四合一) / 4005 (0x280 多电机命令)
    if (tick % 2 == 0)
    {
        if (tick % 4 == 0)
        {
            for (int i = 0; i < 4; i++)
                Motor3508.setCAN(static_cast<int16_t>(chassis_output.out_wheel[i]), i + 1);
            Motor3508.sendCAN();
        }
        else
        {
            Motor4005.ctrl_Torque_All(
                static_cast<int16_t>(chassis_output.out_string[0]),
                static_cast<int16_t>(chassis_output.out_string[1]),
                static_cast<int16_t>(chassis_output.out_string[2]),
                static_cast<int16_t>(chassis_output.out_string[3]));
        }

        BoardCommunicationTX();  // CAN2: 5 帧 (sendPacket 内部有重试)
    }
    else
    {
        supercap.sendCAN();      // CAN2: 1 帧，单独占满 3 邮箱不争抢
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

