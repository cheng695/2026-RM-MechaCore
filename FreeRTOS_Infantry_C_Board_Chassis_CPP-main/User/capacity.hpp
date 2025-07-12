#pragma once
#include <cstdint>
#include "can.hpp"

class Capacity {
   public:
    float In_Power;    
    float voltage;     
    float Out_Power;   
    uint8_t State;        
    uint8_t Cmd;
    float buffer1 = 0;
    float buffer2 = 0;
    float buffer3 = 0;
    float Send_buffer[8] = {0};

    float target_power = 0;
    uint16_t charge_power = 0;  //Ä¬ÈÏ45Íß
    bool is_reply_ = false;

    void EnableOutput();
    void SetMaxChargePower(uint16_t more_power);
    void AskVoltage();
    void AskInputPower();

    void Ask_RCIA();
    void SetMaxChargePower_RCIA();
    void SetDischargePower(uint16_t power);
	
	
	//void Receive(CAN_RxHeaderTypeDef& rx_header,uint8_t* buf);
	//void Send(CAN_HandleTypeDef* hcan, uint8_t robot_id, uint16_t power_limit);
};