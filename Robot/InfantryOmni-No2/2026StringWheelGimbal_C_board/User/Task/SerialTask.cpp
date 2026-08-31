#include "SerialTask.hpp"
#include "CommunicationTask.hpp"
#include "RM_RefereeSystem.h"

extern Vision vision;

/* IMU 和遥控器原始串口数据。 */
BSP::IMU::HI12_float HI12;
uint8_t HI12RX_buffer[82];
BSP::REMOTE_CONTROL::RemoteController DT7;
uint8_t DT7Rx_buffer[18];

/* 结构化输入快照。 */
OperatorInput operator_input;

/* 按键状态对象，用于提供按下、上升沿和下降沿等语义。 */
BSP::Key::SimpleKey Key_x;
BSP::Key::SimpleKey Key_z;
BSP::Key::SimpleKey Key_v;
BSP::Key::SimpleKey Key_b;
BSP::Key::SimpleKey Key_r;
BSP::Key::SimpleKey Key_g;
BSP::Key::SimpleKey Mouse_left;
BSP::Key::SimpleKey Mouse_right;

/* 初始化 IMU 和遥控器串口接收。 */
void SerialInit()
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart1);
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_device(HAL::UART::UartDeviceId::HAL_Uart3);

    HAL::UART::Data uart1_rx_buffer{HI12RX_buffer, sizeof(HI12RX_buffer)};
    HAL::UART::Data uart3_rx_buffer{DT7Rx_buffer, sizeof(DT7Rx_buffer)};

    uart1.receive_dma_idle(uart1_rx_buffer);
    uart3.receive_dma_idle(uart3_rx_buffer);

    uart1.register_rx_callback([](const HAL::UART::Data &data)
    {
        if (data.buffer != nullptr && data.size == sizeof(HI12RX_buffer))
        {
            HI12.DataUpdate(data.buffer);
        }
    });

    uart3.register_rx_callback([](const HAL::UART::Data &data)
    {
        if (data.buffer != nullptr && data.size == sizeof(DT7Rx_buffer))
        {
            DT7.parseData(data.buffer);
        }
    });
}

/* 更新按键对象，使后续输入处理可以使用边沿和持续按下状态。 */
void KeyUpdate()
{
    Key_x.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_X));
    Key_z.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_Z));
    Key_v.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_V));
    Key_b.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_B));
    Key_r.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_R));
    Key_g.update(DT7.get_key(BSP::REMOTE_CONTROL::RemoteController::KEY_G));
    Mouse_left.update(DT7.get_mouseLeft());
    Mouse_right.update(DT7.get_mouseRight());
}

/*
 * 更新每个周期都会变化的输入。
 *
 * vision_enabled 的语义是“当前是否满足进入视觉模式的条件”：
 * 非键鼠挡位只需要视觉系统有效；左中右中的键鼠挡位还必须按住鼠标右键。
 */
void UpdateMomentaryInput(OperatorInput &input)
{
    DT7.SetDeadzone(20.0f);

    input.left_switch = DT7.get_s1();
    input.right_switch = DT7.get_s2();
    input.keyboard_mode =
        input.left_switch == BSP::REMOTE_CONTROL::RemoteController::MIDDLE &&
        input.right_switch == BSP::REMOTE_CONTROL::RemoteController::MIDDLE;

    input.mouse_left = Mouse_left.getPress();
    input.mouse_left_pressed = Mouse_left.getRisingEdge();
    input.mouse_right = Mouse_right.getPress();
    input.key_x = Key_x.getPress();
    input.key_z = Key_z.getPress();
    input.key_v = Key_v.getPress();
    input.key_b = Key_b.getPress();
    input.key_r = Key_r.getPress();
    input.key_g = Key_g.getPress();
    input.mouse_left_released = false;

    input.vision_enabled = vision.getVisionFlag() &&
        (!input.keyboard_mode || input.mouse_right);

    /* 变形请求的具体按键尚未确定，暂时不主动产生折叠请求。 */
}

/* 更新需要跨控制周期保持的输入状态。 */
void UpdateLatchedInput(OperatorInput &input)
{
    static bool launch_enabled = false;
    static bool mouse_left_released_since_press = false;
    /* V 键控制折叠状态：false 表示 UP/MANUAL，true 表示 DOWN。 */
    static bool down_state = false;

    /* 仅在键盘模式下响应 V 键，并使用上升沿实现一次按键切换一次状态。 */
    if (input.keyboard_mode && Key_v.getRisingEdge())
    {
        down_state = !down_state;
    }

    /* 将锁存后的折叠请求交给云台事件检测器。 */
    input.transform_requested = down_state;

    /* B 键是键鼠模式下的发射总使能，每次按下都在 STOP/CEASEFIRE 间翻转。 */
    if (input.keyboard_mode && Key_b.getRisingEdge())
    {
        launch_enabled = !launch_enabled;
    }

    input.launch_enabled = launch_enabled;

    /* 记录鼠标左键是否在本次按下前释放过，供 ONLY/AUTO 切换逻辑使用。 */
    if (Mouse_left.getFallingEdge())
    {
        mouse_left_released_since_press = true;
    }

    input.mouse_left_released = mouse_left_released_since_press;

    if (Mouse_left.getRisingEdge())
    {
        mouse_left_released_since_press = false;
    }

    /* 视觉模式下，发射模式由视觉端下发；非视觉模式下由 B 键切换。 */
    if (input.vision_enabled)
    {
        const uint8_t vision_mode = vision.getVisionMode();
        input.fire_single = vision_mode == 2;
        input.fire_continuous = vision_mode == 1;
    }
    else
    {
        /* 键鼠模式下由 Z/X 选择单发或连发，B 未使能时由事件检测器停火。 */
        input.fire_single = input.key_z;
        input.fire_continuous = input.key_x;
    }

}

/* 返回当前操作员输入快照，供需要一次性读取输入的模块使用。 */
OperatorInput BuildOperatorInput()
{
    OperatorInput input = operator_input;
    UpdateMomentaryInput(input);
    UpdateLatchedInput(input);
    return input;
}

extern "C"
{
void Serival(void const *argument)
{
    (void)argument;
    SerialInit();

    for (;;)
    {
        KeyUpdate();
        UpdateMomentaryInput(operator_input);
        UpdateLatchedInput(operator_input);
        osDelay(5);
    }
}
}
