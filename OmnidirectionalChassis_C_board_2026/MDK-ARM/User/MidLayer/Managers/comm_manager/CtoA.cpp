#include "CtoA.hpp"

BoardComm::CtoA Cboard;

void BoardComm::CtoA::TransmitDataUpdata()
{
    RemoteHandle();
    GetChassisModel();

    auto temp_ptr = tx_Data;

    *temp_ptr = head;
    temp_ptr++;
    auto memcpy_safe = [&temp_ptr](const void *data, size_t size) 
    {
        std::memcpy(temp_ptr, data, size);
        temp_ptr += size;
    };

    memcpy_safe(&direction, sizeof(direction));    // 序列化方向数据
    memcpy_safe(&ChassisModel, sizeof(ChassisModel)); // 序列化模式数据

    uart1_driver.sendData(tx_Data, txByte);
}

void BoardComm::CtoA::ReceiveDataUpdata(const uint8_t *rx_Data)
{
    if(rx_Data[0] != head)
    {
        return;
    }
    else 
    {
        memcpy(&yaw_position, rx_Data + 1, sizeof(yaw_position));
        this->lasttime = this->getlastTime();
    }
}