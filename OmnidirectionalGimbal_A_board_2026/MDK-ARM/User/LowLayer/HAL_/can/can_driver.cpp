#include "can_driver.hpp"

// 全局实例指针（工厂函数中设置）实例化
static CanDriver::CanHal* can1Hal = nullptr;
static CanDriver::CanHal* can2Hal = nullptr;

// 工厂函数
 std::unique_ptr<CanDriver::CanHal> CanDriver::CreateCANHal(uint32_t index)
 {
     switch (index) 
     {
         case 1:
             if (!can1Hal) 
             {
                std::unique_ptr<CanDriver::CanHal> hal(new CanDriver::STM32CANHal);
                can1Hal = hal.get();
                hal->hcan_ = hcan1;               
                hal->ConfigureFilter();
                return hal;
             }
             break;
         case 2:
             if (!can2Hal) 
             {
                std::unique_ptr<CanDriver::CanHal> hal(new CanDriver::STM32CANHal);
                can2Hal = hal.get();
                hal->hcan_ = hcan2;
                hal->ConfigureFilter();
                return hal;
             }
             break;
     }
     return nullptr;
 }

void CanDriver::CanHal::ConfigureFilter()
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;         //启动过滤器
    sFilterConfig.FilterBank           = 0;                         //使用过滤器0
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;          //缓冲器0 有0，1两个
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;     //掩码模式
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;     //处理标准ID和扩展ID
    sFilterConfig.FilterIdHigh         = 0x00;                      //以下四行表示接受所有ID
    sFilterConfig.FilterIdLow          = 0x00;
    sFilterConfig.FilterMaskIdHigh     = 0x00;
    sFilterConfig.FilterMaskIdLow      = 0x00;
    sFilterConfig.SlaveStartFilterBank = 14;    

    if (hcan_.Instance == CAN1)
    {
        sFilterConfig.FilterBank = 0;
        sFilterConfig.SlaveStartFilterBank = 14;
        
        if(HAL_CAN_ConfigFilter(&hcan_, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }
    } 
    else if (hcan_.Instance == CAN2) 
    {
        sFilterConfig.FilterBank = 14;
        // CAN2没有SlaveStartFilterBank设置
        
        if(HAL_CAN_ConfigFilter(&hcan_, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }
    
    // 启动CAN并激活中断
    if (HAL_CAN_Start(&hcan_) != HAL_OK)
    {
        Error_Handler();
    }
    
    if (HAL_CAN_ActivateNotification(&hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) //中断回调函数
{
    if (hcan->Instance == CAN1) CAN1_RxFifo0Callback();         
    else if (hcan->Instance == CAN2) CAN2_RxFifo0Callback();
}

// C→C++桥接函数
void CAN1_RxFifo0Callback(void) //用户中断函数
{ 
    if (can1Hal) //实例化的can
    {
        can1Hal->IrqRxFifo0(); 
    }
}

void CAN2_RxFifo0Callback(void) //用户中断函数
{ 
    if (can2Hal) //实例化的can
    {
        can2Hal->IrqRxFifo0(); 
    } 
}

// 中断处理核心实现
void CanDriver::CanHal::IrqRxFifo0()
{
    CanFrame frame;
    CAN_RxHeaderTypeDef header;
    
    // 从寄存器读取一帧（零拷贝）
    if (HAL_CAN_GetRxMessage(&hcan_, CAN_RX_FIFO0, &header, frame.data) == HAL_OK)
    {
        frame.id = (header.IDE == CAN_ID_EXT) ? header.ExtId : header.StdId;
        frame.dlc = header.DLC;
        frame.isExtended = (header.IDE == CAN_ID_EXT);
        frame.isRemote = (header.RTR == CAN_RTR_REMOTE);
        
        // 立即传递给业务层（中断上下文，要快！）
        if (rxCallback_) 
        {
            rxCallback_(frame);// 由于RmMotorRxData被赋值在rxCallback_之中，因此这里先当与调用RmMotorRxData（frame）也就是说传参是frame
        }
    }
}

