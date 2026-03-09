#include "SerialTask.hpp"
#include "../core/APP/Referee/RM_RefereeSystem.h"

BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];
uint8_t power_buffer[12];
uint8_t referee_buffer[512];
Power power;

namespace
{
constexpr uint16_t kRefereeRxRingSize = 2048;
uint8_t referee_rx_ring[kRefereeRxRingSize];
volatile uint16_t referee_rx_head = 0;
volatile uint16_t referee_rx_tail = 0;

inline void RefereeRxPushByte(uint8_t byte)
{
    uint16_t next = static_cast<uint16_t>((referee_rx_head + 1U) % kRefereeRxRingSize);
    if (next == referee_rx_tail)
    {
        // Drop oldest byte on overflow to keep stream moving.
        referee_rx_tail = static_cast<uint16_t>((referee_rx_tail + 1U) % kRefereeRxRingSize);
    }
    referee_rx_ring[referee_rx_head] = byte;
    referee_rx_head = next;
}

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
} // namespace

/* 按键 ---------------------------------------------------------------------------------------------------*/
bool alphabet[28];  // ctrl 28, shift 27
BSP::Key::SimpleKey Key_w;
BSP::Key::SimpleKey Key_s;
BSP::Key::SimpleKey Key_a;
BSP::Key::SimpleKey Key_d;
BSP::Key::SimpleKey Key_q;
BSP::Key::SimpleKey Key_x;
BSP::Key::SimpleKey Key_r;
BSP::Key::SimpleKey Key_f;
BSP::Key::SimpleKey Key_shift;
BSP::Key::SimpleKey Key_ctrl;
BSP::Key::SimpleKey Mouse_left;
BSP::Key::SimpleKey Mouse_right;

/* 设备通讯回调与数据解析 -------------------------------------------------------------------------------------*/
void SerivalInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);    // 裁判系统
    // auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    
    HAL::UART::Data uart1_rx_buffer{referee_buffer, sizeof(referee_buffer)};
    // HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};

    uart1.receive_dma_idle(uart1_rx_buffer);
    // uart3.receive_dma_idle(uart3_rx_buffer);

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
    // uart3.register_rx_callback([](const HAL::UART::Data &data) 
    // {
    //     if(data.size == 18 && data.buffer != nullptr)
    //     {
    //         DT7.parseData(data.buffer);
    //     }
    // });
}

/* 键鼠逻辑处理 ---------------------------------------------------------------------------------------------*/
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

void KeyProcess(bool *alphabet)
{
    alphabet[22] = Key_w.getPress();    // 前（按下）
    alphabet[18] = Key_s.getPress();    // 后（按下）
    alphabet[0]  = Key_a.getPress();    // 左（按下）
    alphabet[3]  = Key_d.getPress();    // 右（按下）
    alphabet[26] = Key_shift.getPress();    // 超电（按下）
    alphabet[23] = Key_x.getPress();    // X小陀螺（按下）

    alphabet[16] = Key_q.getToggleState();  // 底盘是否跟随（单点）默认跟随

    // Ctrl键 主动补电模式（单点，与Shift/左键/右键互锁）
    if (Key_ctrl.getRisingEdge())
    {
        alphabet[27] = !alphabet[27];
    }
    // 如果按下了加速(Shift)或开火(左键/右键)，则强制退出补电模式
    if (Key_shift.getPress() || Mouse_left.getPress() || Mouse_right.getPress())
    {
        alphabet[27] = false;
    }

    if(Key_ctrl.getPress() && Key_shift.getPress() && Key_f.getPress())
    {
        alphabet[5] = true; // 不信任裁判系统
        alphabet[17] = false;
    }
    else if(Key_ctrl.getPress() && Key_shift.getPress() && Key_r.getPress())
    {
        alphabet[17] = true; // 信任裁判系统
        alphabet[5] = false;
    }
    
}

extern "C" {
void Serival(void const * argument)
{
    SerivalInit();
    for(;;)
    {
        KeyUpdate();
        KeyProcess(alphabet);

        // Parse referee bytes in task context to keep UART ISR short.
        uint8_t byte = 0;
        uint16_t budget = 256;
        while (budget-- > 0 && RefereeRxPopByte(byte))
        {
            RM_RefereeSystem::RM_RefereeSystemParse(&byte);
        }

        osDelay(1);
    }
}

}
