#include "djimotor.hpp"

// 实例化电机
static uint8_t motor_rx_ids[4] = {1, 2, 3, 4};
motor::GM3508<4> Motor3508(0x200, &motor_rx_ids, 0x200);

//can接收回调函数
void GlobalRmMotorRxData(const CanDriver::CanFrame& frame)
{
    Motor3508.RmMotorRxData(frame);
}

// 全局CAN实例
CanDriver::CanHal* can1 = nullptr;
CanDriver::CanHal* can2 = nullptr;

// 初始化can函数
extern "C" void CanInit()
{
    // 1. 创建CAN实例
    auto can1_ptr = CanDriver::CreateCANHal(1);
    auto can2_ptr = CanDriver::CreateCANHal(2);
    
    // 2. 保存指针
    can1 = can1_ptr.get();
    can2 = can2_ptr.get();
    
    // 3. 设置CAN实例到电机对象
    Motor3508.SetCan(can1);
    
    // 4. 设置接收回调函数
    if (can1) can1->SetRxCallback(GlobalRmMotorRxData);
    // if (can2) can2->SetRxCallback(GlobalRmMotorRxData);
    
    // 5. 保持智能指针存活（根据实际情况调整）
    static std::unique_ptr<CanDriver::CanHal> can1_keeper = std::move(can1_ptr);
    static std::unique_ptr<CanDriver::CanHal> can2_keeper = std::move(can2_ptr);
}
