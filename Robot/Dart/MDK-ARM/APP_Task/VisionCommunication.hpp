#ifndef __VisionCommunication_Hpp
#define __VisionCommunication_Hpp

#define VisionSendDelay         27 // 发送频率,单位为赫兹(Hz)

#define VisionUartHandle        huart8
#define VisionUartInstance      UART8
#define VisionUartReceiveLength 6

// 发送类
typedef class VisionUartSendClass
{
public:
    unsigned char Head;
    int Yaw_Temp; // 视觉发送临时变量
} VisionUartSend_t;

// 接收类
typedef class VisionUartReceiveClass
{
public:
    unsigned char ReceiveArr[VisionUartReceiveLength];
    int Yaw_Origin;          // 视觉接收原始变量
    int YawErr;              // 视觉接收误差
    int Target_Yaw;          // 视觉接收目标Yaw角度
    bool isVisionMisaligned; // 表示视觉接收的数据帧错位
    void GetData();          // 获取数据函数
    void FixFrameError(); // 修复数据帧错位函数
} VisionUartReceive_t;

extern VisionUartSend_t VisionUartSend;
extern VisionUartReceive_t VisionUartReceive;

#endif
