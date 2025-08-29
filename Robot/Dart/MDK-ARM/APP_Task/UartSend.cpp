#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 ===========================  */

/*  =========================== 进程的变量 ===========================  */

/*  =========================== 函数的声明 ===========================  */
// void GimbalToChassisUartSend();
void VofaUartSend();

/* Private application code --------------------------------------------------*/
void UartSend(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    /* Infinite loop */
    for (;;) {
        // GimbalToChassisUartSend();
        //VofaUartSend();

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(5));
    }
    /* USER CODE END LED_Flashing */
}
/* Private application code --------------------------------------------------*/

/************************************************************************
 * @brief:      	void
 * @param[in]: 	void
 * @retval:      void
 * @details:    	void
 *************************************************************************/
unsigned char GimbalToChassisUartArr[14] = {0};
// void GimbalToChassisUartSend()
// {
//     GimbalToChassisUartArr[0] = 0x5A;
//     GimbalToChassisUartArr[1] = 0xA5;
//     GimbalToChassisUartArr[2] = 0xAF;
//     GimbalToChassisUartArr[3] = GimbalToChassisUartData.Vx;
//     GimbalToChassisUartArr[4] = GimbalToChassisUartData.Vy;
//     GimbalToChassisUartArr[5] = GimbalToChassisUartData.Vw;
//     memcpy(&GimbalToChassisUartArr[6] ,&GimbalToChassisUartData.YawM6020Angle,sizeof(GimbalToChassisUartData.YawM6020Angle));
//     GimbalToChassisUartArr[8] = GimbalToChassisUartData.ChassisRunMode;
//     GimbalToChassisUartArr[9] = GimbalToChassisUartData.RunState1;
//     GimbalToChassisUartArr[10] = GimbalToChassisUartData.GyroSpeed;
//     GimbalToChassisUartArr[11] = GimbalToChassisUartData.ExtraControlPower;
//     GimbalToChassisUartData.PitchM3508Angle = PitchM3508Data.Angle / 67;
//     memcpy(&GimbalToChassisUartArr[12] ,&GimbalToChassisUartData.PitchM3508Angle,sizeof(GimbalToChassisUartData.PitchM3508Angle));

//     HAL_UART_Transmit_DMA(&ChassisUartHandle,GimbalToChassisUartArr,sizeof(GimbalToChassisUartArr));
// }

/************************************************************************
 * @brief:      	void
 * @param[in]: 	void
 * @retval:      void
 * @details:    	void
 *************************************************************************/
unsigned char VofaUartSendArr[28] = {0}; // 修改数组大小为28字节，以容纳4个float和1个uint32_t

void VofaUartSend()
{
    float Ch1 = RightDownM3508Data.RPM;
    float Ch2 = RightUpM3508Data.RPM;
    float Ch3 = 0 - LeftUpM3508Data.RPM;
    float Ch4 = 0 - LeftDownM3508Data.RPM;

    uint32_t special_value = 0x7f800000;

    memcpy(&VofaUartSendArr[0], &Ch1, sizeof(float));
    memcpy(&VofaUartSendArr[4], &Ch2, sizeof(float));
    memcpy(&VofaUartSendArr[8], &Ch3, sizeof(float));
    memcpy(&VofaUartSendArr[12], &Ch4, sizeof(float));
    memcpy(&VofaUartSendArr[sizeof(float) * 4], &special_value, sizeof(uint32_t));

    HAL_UART_Transmit_DMA(&VofaUartHandle, VofaUartSendArr, sizeof(float) * 5); // 发送的数据长度相应调整
}
