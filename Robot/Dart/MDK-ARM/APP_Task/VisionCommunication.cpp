#include "InHpp.hpp"

/*  =========================== 全局变量的初始化 ===========================  */
VisionUartSend_t VisionUartSend       = {0};
VisionUartReceive_t VisionUartReceive = {0};
unsigned char Arr_VisionSend[6]       = {0};
short SendDelay                       = 1000 / VisionSendDelay; // 发送延时，单位为毫秒
short SendDelayCount                  = 0;

/* Private application code --------------------------------------------------*/
/**
**********************************************************************
* @brief:      	VisionSend: 发送数据
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于发送视觉通信的数据
**********************************************************************
**/
void VisionSend(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    VisionUartSend.Head = 0x39; // 设置发送头部标志
    /* Infinite loop */
    for (;;) {
        VisionUartSend.Yaw_Temp = SpeedPID_AngleSensorM3508.Current - Dart_Yaw_Angle_Medium; // 计算Yaw轴角度(当前值-中值)

        Arr_VisionSend[0] = 0x39;
        Arr_VisionSend[1] = 0x39;

        int Yaw_Temp      = ((VisionUartSend.Yaw_Temp) / 22.755556f) * 100.0f; 
        Arr_VisionSend[2] = Yaw_Temp >> 8 * 3;
        Arr_VisionSend[3] = Yaw_Temp >> 8 * 2;
        Arr_VisionSend[4] = Yaw_Temp >> 8 * 1;
        Arr_VisionSend[5] = Yaw_Temp >> 8 * 0;

        SendDelayCount++;
        if (SendDelayCount >= SendDelay) {
            HAL_UART_Transmit_DMA(&VisionUartHandle, Arr_VisionSend, sizeof(Arr_VisionSend));
            SendDelayCount = 0;
        }

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1));
    }
    /* USER CODE END LED_Flashing */
}

/**
**********************************************************************
* @brief:      	GetData: 接收数据
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于接收视觉通信的数据
**********************************************************************
**/
void VisionUartReceiveClass::GetData()
{
    VisionUartReceiveClass::FixFrameError(); // 修复数据帧错位问题

    // 检查接收数据是否完整 (双帧头均为0x39)
    if (VisionUartReceive.ReceiveArr[0] == VisionUartSend.Head && VisionUartReceive.ReceiveArr[1] == VisionUartSend.Head) {
        VisionUartReceive.Yaw_Origin =
            VisionUartReceive.ReceiveArr[2] << 24 |
            VisionUartReceive.ReceiveArr[3] << 16 |
            VisionUartReceive.ReceiveArr[4] << 8 |
            VisionUartReceive.ReceiveArr[5];
        VisionUartReceive.YawErr     = VisionUartReceive.Yaw_Origin * 22.755556f / 100.0f;
        VisionUartReceive.Target_Yaw = SpeedPID_AngleSensorM3508.Current + VisionUartReceive.YawErr; // 计算目标Yaw角度
    }

    HAL_UART_Receive_DMA(&VisionUartHandle, VisionUartReceive.ReceiveArr, VisionUartReceiveLength);
}

/* Private application code --------------------------------------------------*/
/**
**********************************************************************
* @brief:      	FixFrameError: 修正错帧
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于纠正视觉通信的数据
**********************************************************************
**/
void VisionUartReceiveClass::FixFrameError()
{
    // 查找帧头位置
    int header_pos = -1;
    for (int i = 0; i < sizeof(VisionUartReceive.ReceiveArr) - 1; i++) {
        if ((VisionUartReceive.ReceiveArr[i] == 0x39 && VisionUartReceive.ReceiveArr[i + 1] == 0x39) || (VisionUartReceive.ReceiveArr[i] == 0x66 && VisionUartReceive.ReceiveArr[i + 1] == 0x66)) {
            header_pos = i;
            break;
        }
    }

    // 如果找到了帧头，进行数据移位
    if (header_pos != -1) {
        VisionUartReceive.isVisionMisaligned = true; // 标记为错帧
        // 移动有效数据到数组起始位置
        for (int i = 0; i < sizeof(VisionUartReceive.ReceiveArr) - header_pos; i++) {
            VisionUartReceive.ReceiveArr[i] = VisionUartReceive.ReceiveArr[header_pos + i];
        }
        // 填充无效数据为0x00
        for (int i = sizeof(VisionUartReceive.ReceiveArr) - header_pos; i < sizeof(VisionUartReceive.ReceiveArr); i++) {
            VisionUartReceive.ReceiveArr[i] = 0x00;
        }
    } else {
        // 如果未找到帧头，清空数据
        // memset(vision_rx_data, 0x00, sizeof(vision_rx_data));
    }
}
