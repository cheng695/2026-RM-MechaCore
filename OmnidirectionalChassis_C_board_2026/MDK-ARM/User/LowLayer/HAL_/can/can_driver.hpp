#ifndef _CAN_DRIVER_HPP_
#define _CAN_DRIVER_HPP_

#include <can.h>
#include <functional>
#include <memory>

namespace CanDriver
{
    struct CanFrame 
    {
        uint32_t id{};
        uint8_t  dlc{};
        bool     isExtended{};
        bool     isRemote{};
        uint8_t  data[8]{};
    };

    class CanHal
    {
        public:
            virtual ~CanHal() = default;
            CAN_HandleTypeDef hcan_;
            void ConfigureFilter();
            void IrqRxFifo0();
            using RxCallback = std::function<void(const CanFrame&)>; //定义了一个空变量，这个变量是一个函数（其实可以理解成创建一个类型叫RxCallback）
            virtual bool Send(const CanFrame& frame) = 0;
            void SetRxCallback(RxCallback callback) //传参变量是RxCallback callback以及 RmMotorRxData 函数的类型是 RxCallback 的原因是RmMotorRxData函数的传参以及返回类型和void(const CanFrame&)一样
            {
                rxCallback_ = callback; //把callback塞进空变量（理解成RxCallback类型的变量rxCallback_被函数callback赋值）
            }

        private:
            RxCallback rxCallback_; // 这就是我们说的回调函数对象 把一个函数名给到空变量（理解成类型RxCallback定义了一个变量叫rxCallback_）
    };

    class STM32CANHal : public CanHal
    {
        public:
            bool Send(const CanFrame& frame) override 
            {
                CAN_TxHeaderTypeDef txHeader;
                txHeader.StdId = frame.id;
                txHeader.ExtId = frame.id; 
                txHeader.IDE = frame.isExtended ? CAN_ID_EXT : CAN_ID_STD;
                txHeader.RTR = frame.isRemote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
                txHeader.DLC = frame.dlc;
                txHeader.TransmitGlobalTime = DISABLE;

                uint32_t mailbox;
                return HAL_CAN_AddTxMessage(&hcan_, &txHeader, const_cast<uint8_t*>(frame.data), &mailbox) == HAL_OK;
            }


    };

    std::unique_ptr<CanDriver::CanHal> CreateCANHal(uint32_t index);
}

void CAN1_RxFifo0Callback(void);
void CAN2_RxFifo0Callback(void);

#endif
