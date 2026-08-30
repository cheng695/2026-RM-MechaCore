#include "SerialTask.hpp"
#include "RobotContext.hpp"
#include "RM_RefereeSystem.h"

uint8_t DT7Rx_buffer[18];                  /* 遥控器原始数据缓存 */
uint8_t power_buffer[12];                  /* 外部功率计数据缓存 */
uint8_t referee_buffer[512];               /* 裁判系统串口 DMA 接收缓存 */
Power power;                               /* 外部功率计数据对象 */

namespace
{
constexpr uint16_t kRefereeRxRingSize = 2048; /* 裁判系统字节流环形缓冲区大小 */
uint8_t referee_rx_ring[kRefereeRxRingSize];  /* 中断回调写入、串口任务读出的环形缓冲区 */
volatile uint16_t referee_rx_head = 0;         /* 写指针，指向下一次写入位置 */
volatile uint16_t referee_rx_tail = 0;         /* 读指针，指向下一次读出位置 */

/* UART 接收回调中调用：写入一个字节，满时丢弃最旧数据以保留最新数据。 */
inline void RefereeRxPushByte(uint8_t byte)
{
    uint16_t next = static_cast<uint16_t>((referee_rx_head + 1U) % kRefereeRxRingSize);
    if (next == referee_rx_tail)
    {
        referee_rx_tail = static_cast<uint16_t>((referee_rx_tail + 1U) % kRefereeRxRingSize);
    }
    referee_rx_ring[referee_rx_head] = byte;
    referee_rx_head = next;
}

/* 串口任务中调用：弹出一个字节，缓冲区为空时返回 false。 */
inline bool RefereeRxPopByte(uint8_t &byte)
{
    if (referee_rx_tail == referee_rx_head)
    {
        return false;
    }
    byte = referee_rx_ring[referee_rx_tail];
    referee_rx_tail = static_cast<uint16_t>((referee_rx_tail + 1U) % kRefereeRxRingSize);
    return true;
}
} /* namespace */

BSP::Key::SimpleKey Key_w;           /* 前进 */
BSP::Key::SimpleKey Key_s;           /* 后退 */
BSP::Key::SimpleKey Key_a;           /* 左移 */
BSP::Key::SimpleKey Key_d;           /* 右移 */
BSP::Key::SimpleKey Key_q;           /* 跟随/不跟随切换 */
BSP::Key::SimpleKey Key_x;           /* 小陀螺 */
BSP::Key::SimpleKey Key_r;           /* 恢复信任裁判系统 */
BSP::Key::SimpleKey Key_f;           /* 不信任裁判系统 */
BSP::Key::SimpleKey Key_shift;       /* 加速/超电 */
BSP::Key::SimpleKey Key_ctrl;        /* 主动补电切换组合键 */
BSP::Key::SimpleKey Mouse_left;      /* 鼠标左键 */
BSP::Key::SimpleKey Mouse_right;     /* 鼠标右键 */

/* 初始化裁判系统串口接收。回调只搬运字节，协议解析放到任务上下文中执行。 */
void SerivalInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);

    HAL::UART::Data uart1_rx_buffer{referee_buffer, sizeof(referee_buffer)};

    uart1.receive_dma_idle(uart1_rx_buffer);

    uart1.register_rx_callback([](const HAL::UART::Data &data)
    {
        if(data.size > 0 && data.buffer != nullptr)
        {
            for(uint16_t i = 0; i < data.size; i++)
            {
                RefereeRxPushByte(data.buffer[i]);
            }
        }
    });
}

/* 刷新按键边沿、按下和 toggle 状态。 */
void KeyUpdate()
{
    Key_w.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_W));
    Key_s.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_S));
    Key_a.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_A));
    Key_d.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_D));
    Key_q.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_Q));
    Key_x.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_X));
    Key_r.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_R));
    Key_f.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_F));
    Key_shift.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_SHIFT));
    Key_ctrl.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_CTRL));
    Mouse_left.update(DT7.get_mouseLeft());
    Mouse_right.update(DT7.get_mouseRight());
}

/* 更新摇杆、按键、开关这类每周期直接刷新的瞬时输入。 */
void UpdateMomentaryInput(OperatorInput &input)
{
    DT7.SetDeadzone(20.0f);

    input.left_switch = DT7.get_s1();
    input.right_switch = DT7.get_s2();
    input.keyboard_mode = (input.left_switch == BSP::REMOTE_CONTROL::RemoteController::MIDDLE &&
                           input.right_switch == BSP::REMOTE_CONTROL::RemoteController::MIDDLE);
    input.spin = Key_x.getPress();
    input.boost = Key_shift.getPress();
    input.follow_enabled = Key_q.getToggleState();

    if (input.keyboard_mode)
    {
        /* 键鼠模式：W/S 控制前后，A/D 控制左右，X 触发小陀螺。 */
        input.move_x = static_cast<float>(Key_w.getPress()) - static_cast<float>(Key_s.getPress());
        input.move_y = static_cast<float>(Key_d.getPress()) - static_cast<float>(Key_a.getPress());
        input.rotate = input.spin ? 1.0f : 0.0f;
    }
    else
    {
        /* 遥控器模式：左摇杆控制平移，拨轮控制旋转。 */
        input.move_x = -DT7.get_left_x();
        input.move_y = DT7.get_left_y();
        input.rotate = DT7.get_scroll_();
    }
}

/* 更新 Ctrl 补电、裁判系统信任开关这类需要跨周期保持的锁存输入。 */
void UpdateLatchedInput(OperatorInput &input)
{
    /* Ctrl 上升沿切换主动补电模式。 */
    if (Key_ctrl.getRisingEdge())
    {
        input.charge = !input.charge;
    }

    /* 加速或开火时退出主动补电，避免高功率动作和补电模式同时保持。 */
    if (Key_shift.getPress() || Mouse_left.getPress() || Mouse_right.getPress())
    {
        input.charge = false;
    }

    /* Ctrl + Shift + F/R 用于手动切换是否信任裁判系统数据。 */
    if(Key_ctrl.getPress() && Key_shift.getPress() && Key_f.getPress())
    {
        input.force_referee_untrusted = true;
        input.force_referee_trusted = false;
    }
    else if(Key_ctrl.getPress() && Key_shift.getPress() && Key_r.getPress())
    {
        input.force_referee_trusted = true;
        input.force_referee_untrusted = false;
    }
}

/* 保留 BuildOperatorInput 接口，供需要临时快照的代码使用。 */
OperatorInput BuildOperatorInput()
{
    OperatorInput input = operator_input;
    UpdateMomentaryInput(input);
    return input;
}

/* 更新全局操作员输入快照。 */
void KeyProcess()
{
    UpdateMomentaryInput(operator_input);
    UpdateLatchedInput(operator_input);
}

extern "C" {
void Serival(void const * argument)
{
    SerivalInit();
    for(;;)
    {
        /* 先更新输入快照，再解析裁判系统，保证控制任务读到的是最新操作状态。 */
        KeyUpdate();
        KeyProcess();

        uint8_t byte = 0;
        uint16_t budget = 256;
        /* 每周期最多解析固定数量字节，避免串口任务长时间占用 CPU。 */
        while (budget-- > 0 && RefereeRxPopByte(byte))
        {
            RM_RefereeSystem::RM_RefereeSystemParse(&byte);
        }

        osDelay(1);
    }
}

}
