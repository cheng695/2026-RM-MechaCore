#ifndef _CTOA_HPP_
#define _CTOA_HPP_ 

#include <math.h>
#include <string.h>
#include "../User/LowLayer/HAL_/uart/uart_driver.hpp"
#include "../User/MidLayer/Managers/state_manager/state.hpp"
#include "../User/MidLayer/Controllers/signal_processing/Remote_target.hpp"

#define Yaw_initangle 6835.0f
#define MY_PI 3.14159265358979323846f
#define txByte (1 + sizeof(Direction) + sizeof(State::model::model_state))

extern State::model chass_model;
extern RemoteTarget::remotetarget dr16_tar;
extern UartDriver::uartDriver uart1_driver;

namespace BoardComm
{
    class CtoA : public State::monitoring
    {
        public:
            void ReceiveDataUpdata(const uint8_t *rx_Data);
            void TransmitDataUpdata();

            void RemoteHandle()
            {
                auto channel_to_uint8 = [](float value) { return (static_cast<uint8_t>(value * 110) + 110); };

                if(ChassisModel != State::model::KEYBOARD)
                {
                    direction.LX = channel_to_uint8(dr16_tar.Getvx_left());
                    direction.LY = channel_to_uint8(dr16_tar.Getvy_left());
                    direction.RX = channel_to_uint8(dr16_tar.Getvx_right());
                    direction.RY = channel_to_uint8(dr16_tar.Getvy_right());
                    direction.W  = channel_to_uint8(dr16_tar.GetRoller());
                }
            }

            void GetChassisModel()
            {
                this->ChassisModel = chass_model.getCurrentState();
            }

            void Theta_encoder_angle_err()
            {
                Yaw_encoder_angle_err = yaw_position.Yaw_angle - Yaw_initangle;
                Yaw_encoder_angle_err = Yaw_encoder_angle_err / 8191.0f * 360.0f * MY_PI /180.0f;
                Yaw_encoder_angle_err = fmod(Yaw_encoder_angle_err, 2*MY_PI);
                if(Yaw_encoder_angle_err < -MY_PI)
                {
                    Yaw_encoder_angle_err = Yaw_encoder_angle_err + 2*MY_PI;
                }
                else if(Yaw_encoder_angle_err > MY_PI)
                {
                    Yaw_encoder_angle_err = Yaw_encoder_angle_err - 2*MY_PI;
                }
            }

            void checkBoardCommState()
            {
                this->time = this->getTime();
                this->isOnline = this->checkTime(200);
            }

            bool GetIsBoardCommOnline() const
            {
                return this->isOnline;
            }

        private:
            struct __attribute__((packed)) Direction 
            {
                uint8_t LX;
                uint8_t LY;
                uint8_t RX;
                uint8_t RY;
                uint8_t W;
            };   

            struct __attribute__((packed)) YawPosition 
            {
                float Yaw_angle;
            };   

            uint8_t head = 0x55;
            uint8_t tx_Data[txByte];
            
            bool isOline;
            float lasttime;
            float time;
            float Yaw_encoder_angle_err;

            State::model::model_state ChassisModel;

            Direction direction;
            YawPosition yaw_position;
    };
}

#endif
