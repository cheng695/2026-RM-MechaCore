#include "MotorTask.hpp"
#include "ControlTask.hpp"
#include "Lk_motor.hpp"
#include "../../User/core/BSP/Common/CANTransport/CANTransport.hpp"
#include "SerialTask.hpp"

float whatch_motor4005[4] = {0.0f}; // Ozone watch
float whatch_motor4005_velocity[4] = {0.0f}; // Ozone watch

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<4> Motor3508(0x200, {1, 2, 3, 4}, 0x200);
BSP::Motor::LK::LK4005<4> Motor4005(0x140, {1, 2, 3, 4}, {1, 2, 3, 4});

/* 板间CAN通讯接收缓冲区 ----------------------------------------------------------------------------------*/
BSP::CANTransport::RxBuffer board_downlink_rx;  // 收云台下发的数据 (CAN ID 0x311)

/* CAN2 上行发送缓存 --------------------------------------------------------------------------------------*/
static BoardCommunication::BoardTX uplink_tx{}; // C板→云台 上行数据

/*CAN接收 ---------------------------------------------------------------------------------------------*/
void MotorInit(void)
{
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    static auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);

    can1.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        Motor3508.Parse(frame);
        Motor4005.Parse(frame);
    });
    can2.register_rx_callback([](const HAL::CAN::Frame &frame)
    {
        supercap.Parse(frame);
        if (frame.id == 0x311) board_downlink_rx.feed(frame);  // 板间通讯下行: CAN ID 0x311
    });
}

/**
 * @brief 处理云台下发的板间数据 (CAN ID 0x311)
 *
 * CANTransport 拼包完成后解析 BoardRX 结构体 (35 字节)。
 */
static void process_board_downlink()
{
    if (board_downlink_rx.complete())
    {
        __disable_irq();  // 防止 CAN ISR 在拷贝期间进入 feed() 清空 buffer
        memcpy(&Cboard.board_rx_, board_downlink_rx.data(), sizeof(Cboard.board_rx_));
        board_downlink_rx.reset();
        __enable_irq();

        Cboard.updateTimestamp();
        DT7.parseData(Cboard.board_rx_.rc_data);
    }
}

/**
 * @brief 板间通讯 CAN 发送 (C板 → 云台)
 *
 * 将 BoardTX (28 字节) 通过 CAN2 分包发送。
 * CAN ID: 0x310, ceil(28/7) = 4 帧。
 */
static void BoardCommunicationTX()
{
    auto &can2 = HAL::CAN::get_can_bus_instance().get_can2();

    // 里程计
    uplink_tx.x   = navigation.X;
    uplink_tx.y   = navigation.Y;
    uplink_tx.yaw = navigation.Yaw;

    // 速度
    uplink_tx.vx = static_cast<float>(string_fk.GetChassisVx());
    uplink_tx.vy = static_cast<float>(string_fk.GetChassisVy());
    uplink_tx.wz = static_cast<float>(string_fk.GetChassisVw());

    // 裁判系统
    uplink_tx.shooter_barrel_cooling_value = ext_power_heat_data_0x0201.shooter_barrel_cooling_value;
    uplink_tx.shooter_barrel_heat_limit    = ext_power_heat_data_0x0201.shooter_barrel_heat_limit;

    BSP::CANTransport::sendPacket(can2, 0x310, &uplink_tx, sizeof(uplink_tx));
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
float motor3508;
extern "C"{
void Motor(void const * argument)
{
    MotorInit();
    static uint32_t loop_count = 0;
    for(;;)
    {
        loop_count++;
        motor_control_logic(loop_count);
        for(int i = 0; i < 4; i++)
        {
            whatch_motor4005[i] = Motor4005.getAngleDeg(i+1); // Ozone watch
            whatch_motor4005_velocity[i] = Motor4005.getVelocityRpm(i+1); // Ozone watch
        }
        motor3508 = Motor3508.getVelocityRpm(1) * (268.0f/17.0f); // Ozone watch
        osDelay(1);
    } 
}

}

