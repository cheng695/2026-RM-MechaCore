#include "InHpp.hpp"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == DT7UartInstance)
        DT7UartCom.GetMessage();
    // if (huart->Instance == ChassisUartInstance)
    //     ChassisUartCom.GetData();
    // if(huart->Instance == HI14UartInstance)
    // HI14UartCom.GetData();
    if (huart->Instance == HuartDistance_RMRefereeSystem)
        MyRefereeSystemParse();
    if (huart->Instance == VisionUartInstance)
        VisionUartReceive.GetData();
}

// 空闲中断回调函数
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == VofaUartInstance) {
        // 处理接收到的数据
        VofaCallBack.ProcessReceivedData();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE) {
        // if(huart->Instance == HI14UartInstance)
        // __HAL_UART_CLEAR_OREFLAG(&huart7);
        if (huart->Instance == USART6)
            __HAL_UART_CLEAR_OREFLAG(&huart6);
        if (huart->Instance == UART8)
            __HAL_UART_CLEAR_OREFLAG(&huart8);
    }
}
