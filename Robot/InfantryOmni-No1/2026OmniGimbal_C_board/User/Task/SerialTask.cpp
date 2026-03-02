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
bool last_is_vision = false; // 上一次是不是视觉模式
bool alphabet[28];  // 27：鼠标左键，28：鼠标右键
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

/* 键鼠逻辑 ---------------------------------------------------------------------------------------------*/
/**
 * @brief 原始状态更新，放置于SimpleKey中
 * 
 */
void KeyUpdate()
{
    Key_z.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_Z));
    Key_x.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_X));
    Key_b.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_B));
    Mouse_left.update(DT7.get_mouseLeft());
    Mouse_right.update(DT7.get_mouseRight());
}

/**
 * @brief 键鼠逻辑处理
 * 
 * @param alphabet 
 */
void KeyProcess(bool *alphabet)
{
    alphabet[26] = Mouse_left.getPress();
    alphabet[27] = Mouse_right.getPress();

    // 鼠标左键有没有松开过
    is_change = Mouse_left.getFallingEdge(); 

    // 左键也能开启摩擦轮，B键反转
    static bool friction_wheel_state = false;
    if (Key_b.getRisingEdge()) 
    {
        friction_wheel_state = !friction_wheel_state;
    }
    if (Mouse_left.getRisingEdge()) 
    {
        friction_wheel_state = true;
    }
    alphabet[1] = friction_wheel_state;

    /* --- 1. 判定是否进入视觉托管模式 --- */
    if(vision.getVisionFlag())
    {
        // 如果是键鼠模式 (3,3)，不仅要 visionFlag 还要按住右键
        if(DT7.get_s1() == 3 && DT7.get_s2() == 3) 
        {
            is_vision = alphabet[27]; // 可以直接赋值
        }
        else // 遥控模式，只要 visionFlag 就托管
        {
            is_vision = true;
        }
    }
    else
    {
        is_vision = false;
    }
    // 检测是否刚刚退出视觉模式（下降沿），如果是，为了安全最好重置一下
    if (last_is_vision && !is_vision) { alphabet[23]=0; alphabet[25]=0; } 
    /* --- 2. 执行逻辑 --- */
    if(is_vision)
    {
        // 视觉完全接管 Z/X 状态
        uint8_t mode = vision.getVisionMode();
        if(mode == 0)      { alphabet[25] = false; alphabet[23] = false; }
        else if(mode == 1) { alphabet[25] = true;  alphabet[23] = false; }
        else if(mode == 2) { alphabet[25] = false; alphabet[23] = true;  }
    }
    else
    {
        // 手动逻辑
        if (Key_z.getRisingEdge())
        {
            alphabet[25] = true;
            alphabet[23] = false;
        }
        else if (Key_x.getRisingEdge())
        {
            alphabet[23] = true;
            alphabet[25] = false;
        }
        else if (Mouse_left.getRisingEdge() && launch_fsm.Get_Now_State() == LAUNCH_CEASEFIRE)
        {
            alphabet[23] = true; 
            alphabet[25] = false;   
        }


    }

    last_is_vision = is_vision; 
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
        KeyUpdate();
        KeyProcess(alphabet);
        osDelay(5);
    }
}

}
