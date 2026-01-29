#include "SerialTask.hpp"
#include "../User/Task/ControlTask.hpp"
#include "../User/Task/CommunicationTask.hpp"
extern Launch_FSM launch_fsm; 
extern Vision vision;

/**
 * @brief 初始化
 */
/* 陀螺仪 ---------------------------------------------------------------------------------------------------*/
BSP::IMU::HI12_float HI12;
uint8_t HI12RX_buffer[82];

/* 遥控器 ---------------------------------------------------------------------------------------------------*/
BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];

/* 按键 ---------------------------------------------------------------------------------------------------*/
bool is_change;
bool is_vision;
bool alphabet[28];
BSP::Key::SimpleKey Key_z;
BSP::Key::SimpleKey Key_x;
BSP::Key::SimpleKey Key_b;
BSP::Key::SimpleKey Mouse_left;
BSP::Key::SimpleKey Mouse_right;

/* 串口接收 ---------------------------------------------------------------------------------------------*/
/**
 * @brief 串口初始化函数
 * 
 * 初始化串口并注册设备反馈数据解析回调函数
 */
void SerialInit()
{
    // 实例串口
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);
    
    // 设置缓冲区
    HAL::UART::Data uart1_rx_buffer{HI12RX_buffer, 82};
    HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, 18};

    // 注册串口接收回调函数
    uart1.receive_dma_idle(uart1_rx_buffer);
    uart3.receive_dma_idle(uart3_rx_buffer);
    uart1.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == 82 && data.buffer != nullptr)
        {
            HI12.DataUpdate(data.buffer);
        }
    });
    uart3.register_rx_callback([](const HAL::UART::Data &data) 
    {
        if(data.size == 18 && data.buffer != nullptr)
        {
            DT7.parseData(data.buffer);
        }
    });
}

void KeyUpdate()
{
    Key_z.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_Z));
    Key_x.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_X));
    Key_b.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_B));
    Mouse_left.update(DT7.get_mouseLeft());
    Mouse_right.update(DT7.get_mouseRight());
}

void KeyProcess(bool *alphabet)
{
    alphabet[1] = Key_b.getToggleState();

    if(vision.getVisionFlag())
    {
        if(DT7.get_s1 == 3 && DT7.get_s2 == 3)
        {
            if(Mouse_right.getPress())
            {
                is_vision = true;
            }
        }
        else
        {
            is_vision = true;
        }
    }
    // Z 和 X 的互斥逻辑
    if (Key_z.getRisingEdge())  // 如果 Z 刚被按下
    {
        alphabet[25] = true;    // Z 变真
        alphabet[23] = false;   // X 强制变假
    }
    else if (Key_x.getRisingEdge()) // 如果 X 刚被按下
    {
        alphabet[23] = true;    // X 变真
        alphabet[25] = false;   // Z 强制变假
    }
    else if (Mouse_left.getRisingEdge() && launch_fsm.Get_Now_State() == LAUNCH_CEASEFIRE) // 如果左键按下 且 当前是停火状态
    {
        alphabet[23] = true; 
        alphabet[25] = false;   
    }
}

/* 任务函数 --------------------------------------------------------------------------------------------*/
/**
 * @brief 串口接收任务函数
 * 
 * 任务主循环，任务为空
 * 
 * @param argument 任务参数指针
 */
extern "C" {
void Serial(void const * argument)
{
    SerialInit();
    for(;;)
    {
        osDelay(1);
    }
}

}
