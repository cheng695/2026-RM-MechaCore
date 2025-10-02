#ifndef _UART_DRIVER_HPP_
#define _UART_DRIVER_HPP_ 

#include "usart.h"
#include "dma.h"
#include <functional>
#include <memory>

namespace UartDriver
{
    class uartDriver;
}

void UartDriver_Register(UartDriver::uartDriver* driver, UART_HandleTypeDef* huart);

namespace UartDriver
{
    class uartDriver
    {
        public:
            enum UartMode
            {
                TX_ONLY,
                RX_ONLY,
                TX_RX
            };

            using RxCallback = std::function<void(const uint8_t*, uint16_t)>;
            
            uartDriver(UART_HandleTypeDef* huart, uint16_t tx_size, uint16_t rx_size, UartMode mode)
            {
                huart_ = huart;
                tx_buffer_size = tx_size;
                rx_buffer_size = rx_size;
                work_mode = mode;
                rx_buffer = (work_mode == RX_ONLY || work_mode == TX_RX) ? new uint8_t[rx_size] : nullptr;
                tx_buffer = (work_mode == TX_ONLY || work_mode == TX_RX) ? new uint8_t[tx_size] : nullptr;
                is_uart_online = true;
                rx_data_size = 0;
                
                // 添加调试信息，确认rx_buffer_size的值
                if (rx_buffer && (work_mode == RX_ONLY || work_mode == TX_RX)) {
                    // 可以在这里设置断点或添加日志输出
                }
                
                UartDriver_Register(this, huart_);
            }

            ~uartDriver()
            {
                delete[] rx_buffer;
                delete[] tx_buffer;
            }

            bool UartInit()
            {
                if ((work_mode == TX_ONLY || work_mode == TX_RX) && 
                    HAL_UART_Transmit_DMA(huart_, tx_buffer, tx_buffer_size) != HAL_OK) {
                    return false;
                }
                
                if ((work_mode == RX_ONLY || work_mode == TX_RX) && 
                    HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer, rx_buffer_size) != HAL_OK) {
                    return false;
                }

                is_uart_enable = true;
                return true;
            }

            bool sendData(const uint8_t* data, uint16_t size)
            {
                HAL_UART_Transmit_DMA(huart_, data, size);
                return true;
            }

            void myRxCallback(uint16_t size)
            {
                rx_data_size = size;
                HAL_UARTEx_ReceiveToIdle_DMA(huart_, rx_buffer, rx_buffer_size);
                if (rxCallback_) 
                {
                    rxCallback_(rx_buffer, rx_data_size);
                }
            }

            void myTxCallback()
            {

            }

            void setRxCallback(RxCallback callback)
            {
                rxCallback_ = callback;
            }

            const uint8_t* getRxBuffer() const { return rx_buffer; }
            uint16_t getRxBufferSize() const { return rx_buffer_size; }
            uint16_t getRxDataSize() const { return rx_data_size; }
            UART_HandleTypeDef* getHandle() const { return huart_; }

        private:
            UART_HandleTypeDef* huart_;
            uint8_t* rx_buffer;
            uint8_t* tx_buffer;
            uint16_t tx_buffer_size;
            uint16_t rx_buffer_size;
            uint16_t rx_data_size;
            UartMode work_mode;
            bool is_uart_enable;
            bool is_uart_online;
            RxCallback rxCallback_;
    };
}

#ifdef __cplusplus
extern "C" {
#endif

void RxEventCallback();

#ifdef __cplusplus
}
#endif


#endif
