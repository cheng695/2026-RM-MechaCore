#include "Callback.hpp"

extern UartDriver::uartDriver uart6_driver;

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6) 
{
    uint8_t send_str2[sizeof(float) * 8]; // 分配8个float空间（32字节）
    const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

    // 通过DMA发送完整帧（6数据 + 1帧尾 = 7个float，共28字节）
    uart6_driver.sendData(send_str2, sizeof(float) * 7);
}

