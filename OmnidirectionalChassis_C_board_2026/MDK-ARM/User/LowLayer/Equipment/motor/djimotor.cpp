#include "djimotor.hpp"

//实例化
motor::DJImotor M3508_201(0x201);
motor::DJImotor M3508_202(0x202);
motor::DJImotor M3508_203(0x203);
motor::DJImotor M3508_204(0x204);

void motor::DJImotor::DataUpdate(const uint8_t *data, float Maximum_angle)
{
    ref_angle           =(int16_t)(data[0]<<8|data[1]);       //角度值
    ref_speed           =(int16_t)(data[2]<<8|data[3]);       //速度
    ref_torqueCurrent   =(int16_t)(data[4]<<8|data[5]);       //电流
    ref_temperate       =data[6];  

    angle           = (float)ref_angle;
    speed           = (float)ref_speed;
    torquecurrent   = (float)ref_torqueCurrent;
    
    angle_8191      =(float)ref_angle / Maximum_angle * 8191.0f;
    angle_360       =(float)ref_angle / Maximum_angle * 360.0f;
    speed_rad       =(float)ref_speed * (MY_PI / 30.0f);  
}

void RmMotorRxData(const CanDriver::CanFrame& frame)
{
    switch (frame.id)
    {
        case 0x201:
            M3508_201.DataUpdate(frame.data, 8191.0f);
            M3508_201.getTime();
            //M3508_201.totalAngle = M3508_201.getAccumulatedAngle(8191.0f);
            break;

        case 0x202:
            M3508_202.DataUpdate(frame.data, 8191.0f);
            M3508_202.getTime();
            break;

        case 0x203:
            M3508_203.DataUpdate(frame.data, 8191.0f);
            M3508_203.getTime();
            break;

        case 0x204:
            M3508_204.DataUpdate(frame.data, 8191.0f);
            M3508_204.getTime();
            break;
    }
}

bool motor::DJIMotorController::SendMotorData(uint16_t cmdId, int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    CanDriver::CanFrame frame;
    frame.id = cmdId;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemote = false;
    
    // 打包4个电机的16位数据（每个电机占2个字节）
    PackInt16(&frame.data[0], m1);
    PackInt16(&frame.data[2], m2);
    PackInt16(&frame.data[4], m3);
    PackInt16(&frame.data[6], m4);
    
    // 通过CAN发送
    return can_.Send(frame);
}

//创建对象
extern "C" void CanInit()
{
    // 1. 创建CAN实例
    auto can1 = CanDriver::CreateCANHal(1);
    auto can2 = CanDriver::CreateCANHal(2);
    
    // 2. 设置接收回调函数
    if (can1) can1->SetRxCallback(RmMotorRxData);
    if (can2) can2->SetRxCallback(RmMotorRxData);
    
    // 3. 创建电机控制器（需要在适当的作用域内）
    static motor::DJIMotorController motorController1(*can1);  // 使用static确保生命周期
    static motor::DJIMotorController motorController2(*can2);
}
