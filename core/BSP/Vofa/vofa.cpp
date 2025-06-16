#include "vofa.hpp"

namespace BSP::VOFA
{

Vofa::Vofa(HAL::UART::UartDeviceId uart_id) : uart_id_(uart_id), packet_()
{
    // 数据包在构造函数中已自动初始化
}

Vofa &Vofa::getInstance(HAL::UART::UartDeviceId uart_id)
{
    // 懒汉式单例模式 - 线程安全的静态局部变量
    static Vofa instance(uart_id);
    return instance;
}

void Vofa::justFloat(uint8_t count, ...)
{
    // 限制count不超过最大值
    if (count > MAX_FLOAT_COUNT)
    {
        count = MAX_FLOAT_COUNT;
    }

    // 初始化va_list
    va_list args;
    va_start(args, count);

    // 直接将float参数写入数据包结构体
    for (uint8_t i = 0; i < count; i++)
    {
        packet_.data[i] = static_cast<float>(va_arg(args, double)); // va_arg对float会提升为double
    }

    va_end(args);

    // 计算实际发送的数据大小
    uint16_t data_size = count * sizeof(float) + sizeof(uint32_t);

    // 发送数据包
    HAL::UART::Data uart_data{reinterpret_cast<uint8_t *>(&packet_), data_size};
    HAL::UART::get_uart_bus_instance().get_device(uart_id_).transmit_dma(uart_data);
}

} // namespace BSP::VOFA
