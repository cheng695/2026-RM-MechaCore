#include "can_driver.hpp"

// ȫ��ʵ��ָ�루�������������ã�ʵ����
static std::unique_ptr<CanDriver::STM32CANHal> can1HAL;
static std::unique_ptr<CanDriver::STM32CANHal> can2HAL;

// ��������
 std::unique_ptr<CanDriver::CanHal> CanDriver::CreateCANHal(uint32_t index)
 {
     switch (index) 
     {
         case 1:
             if (!can1HAL) 
             {
                can1HAL = std::unique_ptr<STM32CANHal>(new STM32CANHal());
                can1HAL->hcan_ = hcan1;               
                can1HAL->ConfigureFilter();
             }
             return std::unique_ptr<CanHal>(can1HAL.get());
         case 2:
             if (!can2HAL) 
             {
                can2HAL = std::unique_ptr<STM32CANHal>(new STM32CANHal());
                can2HAL->hcan_ = hcan2;
                can2HAL->ConfigureFilter();
            }
            return std::unique_ptr<CanHal>(can2HAL.get());
     }
     return nullptr;
 }

void CanDriver::CanHal::ConfigureFilter()
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterActivation     = CAN_FILTER_ENABLE;         //����������
    sFilterConfig.FilterBank           = 0;                         //ʹ�ù�����0
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;          //������0 ��0��1����
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;     //����ģʽ
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;     //�����׼ID����չID
    sFilterConfig.FilterIdHigh         = 0x00;                      //�������б�ʾ��������ID
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
        // CAN2û��SlaveStartFilterBank����
        
        if(HAL_CAN_ConfigFilter(&hcan_, &sFilterConfig) != HAL_OK)
        {
            Error_Handler();
        }
    }
    
    // ����CAN�������ж�
    if (HAL_CAN_Start(&hcan_) != HAL_OK)
    {
        Error_Handler();
    }
    
    if (HAL_CAN_ActivateNotification(&hcan_, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) //�жϻص�����
{
    if (hcan->Instance == CAN1) CAN1_RxFifo0Callback();         
    else if (hcan->Instance == CAN2) CAN2_RxFifo0Callback();
}

// C��C++�ŽӺ���
void CAN1_RxFifo0Callback(void) //�û��жϺ���
{ 
    if (can1HAL) //ʵ������can
    {
        can1HAL->IrqRxFifo0(); 
    }
}

void CAN2_RxFifo0Callback(void) //�û��жϺ���
{ 
    if (can2HAL) //ʵ������can
    {
        can2HAL->IrqRxFifo0(); 
    } 
}

// �жϴ������ʵ��
void CanDriver::CanHal::IrqRxFifo0()
{
    CanFrame frame;
    CAN_RxHeaderTypeDef header;
    
    // �ӼĴ�����ȡһ֡���㿽����
    if (HAL_CAN_GetRxMessage(&hcan_, CAN_RX_FIFO0, &header, frame.data) == HAL_OK)
    {
        frame.id = (header.IDE == CAN_ID_EXT) ? header.ExtId : header.StdId;
        frame.dlc = header.DLC;
        frame.isExtended = (header.IDE == CAN_ID_EXT);
        frame.isRemote = (header.RTR == CAN_RTR_REMOTE);
        
        // �������ݸ�ҵ��㣨�ж������ģ�Ҫ�죡��
        if (rxCallback_) 
        {
            rxCallback_(frame);// ����RmMotorRxData����ֵ��rxCallback_֮�У���������ȵ������RmMotorRxData��frame��Ҳ����˵������frame
        }
    }
}

