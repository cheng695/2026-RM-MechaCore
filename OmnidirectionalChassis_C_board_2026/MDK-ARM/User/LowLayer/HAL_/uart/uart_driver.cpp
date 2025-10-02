#include "uart_driver.hpp"
#include "../User/LowLayer/Equipment/remote/Dr16.hpp"

extern Clicker::DR16 dr16;

UartDriver::uartDriver uart3_driver(&huart3, 0, 18, UartDriver::uartDriver::RX_ONLY);
UartDriver::uartDriver uart6_driver(&huart6, 32, 0, UartDriver::uartDriver::TX_ONLY);

static UartDriver::uartDriver* uart_drivers[6] = {nullptr};

void UartDriver_Register(UartDriver::uartDriver* driver, UART_HandleTypeDef* huart)
{
    for (int i = 0; i < 6; i++) 
    {
        if (uart_drivers[i] == nullptr) 
        {
            uart_drivers[i] = driver;
            return;
        }
    }
}

static UartDriver::uartDriver* UartDriver_Find(UART_HandleTypeDef* huart)
{
    for (int i = 0; i < 6; i++) 
    {
        if (uart_drivers[i] != nullptr && uart_drivers[i]->getHandle() == huart) 
        {
            return uart_drivers[i];
        }
    }
    return nullptr;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    UartDriver::uartDriver* driver = UartDriver_Find(huart);
    if (driver != nullptr) 
    {
        driver->myRxCallback(Size);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
    {     
        if(huart->Instance == USART3)
        __HAL_UART_CLEAR_OREFLAG(&huart3);
        if(huart->Instance == USART1)
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
}

// DR16数据接收回调函数
void DR16_RxCallback(const uint8_t* data, uint16_t size) 
{
    if (size == 18) 
    {
        dr16.DR16DataUpdata(data);
    }
}

extern "C" void RxEventCallback()
{
    uart3_driver.setRxCallback(DR16_RxCallback);
    uart3_driver.UartInit();
    uart6_driver.UartInit();
}
