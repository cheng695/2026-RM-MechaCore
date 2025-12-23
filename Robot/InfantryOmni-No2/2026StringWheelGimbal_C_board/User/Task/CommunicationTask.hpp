#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"
#include "../user/core/BSP/Common/StateWatch/state_watch.hpp"
#include "../user/core/BSP/Common/StateWatch/buzzer_manager.hpp"

extern uint8_t CommunicationData[18];;

class BoardCommunication
{
    public:
        BoardCommunication(int timeThreshold = 100) : statewatch_(timeThreshold) 
        {
        }
        virtual ~BoardCommunication() = default;

        void updateTimestamp()
        {
            statewatch_.UpdateLastTime();
        }

        bool isConnected()
        {
            statewatch_.UpdateTime();
            statewatch_.CheckStatus();
            if(statewatch_.GetStatus() == BSP::WATCH_STATE::Status::OFFLINE)
            {
                BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing();
            }
            return statewatch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
        }

    private:
        BSP::WATCH_STATE::StateWatch statewatch_;
};

#endif
