#include "SerialTask.hpp"
#include "../core/APP/Referee/RM_RefereeSystem.h"

BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];
uint8_t referee_buffer[1];

uint8_t power_buffer[12];
Power power;

/* 按键 ---------------------------------------------------------------------------------------------------*/
bool alphabet[28];
BSP::Key::SimpleKey Key_w;
BSP::Key::SimpleKey Key_s;
BSP::Key::SimpleKey Key_a;
BSP::Key::SimpleKey Key_d;
BSP::Key::SimpleKey Key_q;
BSP::Key::SimpleKey Key_e;
BSP::Key::SimpleKey Key_c;
BSP::Key::SimpleKey Key_v;
BSP::Key::SimpleKey Key_shift;
BSP::Key::SimpleKey Key_ctrl;

/* 设备通讯回调与数据解析 -------------------------------------------------------------------------------------*/
void SerivalInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);    // 裁判系统
    //auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);  // 遥控器
    
    HAL::UART::Data uart1_rx_buffer{referee_buffer, 1};
    //HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};

    uart1.receive(uart1_rx_buffer);
    //uart3.receive_dma_idle(uart3_rx_buffer);

    uart1.register_rx_callback([](const HAL::UART::Data &data) 
    {

        if(data.size == 1 && data.buffer != nullptr)
        {
            RM_RefereeSystem::RM_RefereeSystemParse(data.buffer);
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
    Key_e.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_E));
    Key_c.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_C));
    Key_v.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_V));
    Key_shift.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_SHIFT));
    Key_ctrl.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_CTRL));
}

void KeyProcess(bool *alphabet)
{
    alphabet[22] = Key_w.getPress();    // 前（按下）
    alphabet[18] = Key_s.getPress();    // 后（按下）
    alphabet[0]  = Key_a.getPress();    // 左（按下）
    alphabet[3]  = Key_d.getPress();    // 右（按下）
    alphabet[26] = Key_shift.getPress();    // 超电（按下）

    if (Key_q.getRisingEdge())  // 向左小陀螺（单点，再点停下）
    {
        alphabet[16] = !alphabet[16];
        if(alphabet[16]) alphabet[4] = false;
    }
    if (Key_e.getRisingEdge())  // 向右小陀螺（单点，再点停下）
    {
        alphabet[4] = !alphabet[4];
        if(alphabet[4]) alphabet[16] = false;
    }

    if (Key_c.getRisingEdge())  // 底盘跟随（单点）
    {
        alphabet[2] = !alphabet[2];
        if(alphabet[2]) alphabet[21] = false;
    }
    if (Key_v.getRisingEdge())  // 底盘不跟随（单点）
    {
        alphabet[21] = !alphabet[21];
        if(alphabet[21]) alphabet[2] = false;
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
        osDelay(1);
    }
}

}
