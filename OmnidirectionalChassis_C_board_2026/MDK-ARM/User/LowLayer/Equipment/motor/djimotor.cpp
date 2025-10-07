#include "djimotor.hpp"

// ʵ�������
static uint8_t motor_rx_ids[4] = {1, 2, 3, 4};
motor::GM3508<4> Motor3508(0x200, &motor_rx_ids, 0x200);

//can���ջص�����
void GlobalRmMotorRxData(const CanDriver::CanFrame& frame)
{
    Motor3508.RmMotorRxData(frame);
}

// ȫ��CANʵ��
CanDriver::CanHal* can1 = nullptr;
CanDriver::CanHal* can2 = nullptr;

// ��ʼ��can����
extern "C" void CanInit()
{
    // 1. ����CANʵ��
    auto can1_ptr = CanDriver::CreateCANHal(1);
    auto can2_ptr = CanDriver::CreateCANHal(2);
    
    // 2. ����ָ��
    can1 = can1_ptr.get();
    can2 = can2_ptr.get();
    
    // 3. ����CANʵ�����������
    Motor3508.SetCan(can1);
    
    // 4. ���ý��ջص�����
    if (can1) can1->SetRxCallback(GlobalRmMotorRxData);
    // if (can2) can2->SetRxCallback(GlobalRmMotorRxData);
    
    // 5. ��������ָ�������ʵ�����������
    static std::unique_ptr<CanDriver::CanHal> can1_keeper = std::move(can1_ptr);
    static std::unique_ptr<CanDriver::CanHal> can2_keeper = std::move(can2_ptr);
}
