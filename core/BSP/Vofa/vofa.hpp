/**
 * @file vofa.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief VOFA
 * @version 0.0.1
 * @date 2025-06-16
 *
 * @copyright SZPU-RCIA (c) 2025
 *
 */

#include "core/HAL/UART/uart_hal.hpp"
#include <stdarg.h> // 用于va_list

namespace BSP::VOFA
{

// VOFA JustFloat协议数据包结构体
struct __attribute__((packed)) VofaSendPacket
{
    float data[8];        // 最大8个float数据
    uint32_t tail_frame_; // 协议尾帧

    VofaSendPacket() : tail_frame_(0x7f800000)
    {
        // 初始化数据数组
        for (int i = 0; i < 8; i++)
        {
            data[i] = 0.0f;
        }
    }
};
class Vofa
{
  private:
    HAL::UART::UartDeviceId uart_id_;
    VofaSendPacket packet_;                       // VOFA数据包
    static constexpr uint8_t MAX_FLOAT_COUNT = 8; // 最大支持8个float

    // 私有构造函数
    explicit Vofa(HAL::UART::UartDeviceId uart_id);

    // 禁用拷贝构造函数和赋值操作符
    Vofa(const Vofa &) = delete;
    Vofa &operator=(const Vofa &) = delete;

  public:
    // 获取单例实例（懒汉模式）
    static Vofa &getInstance(HAL::UART::UartDeviceId uart_id = HAL::UART::UartDeviceId::HAL_Uart1);

    // 使用va_list的可变参数版本 (count指定参数个数)
    void justFloat(uint8_t count, ...);
};
} // namespace BSP::VOFA