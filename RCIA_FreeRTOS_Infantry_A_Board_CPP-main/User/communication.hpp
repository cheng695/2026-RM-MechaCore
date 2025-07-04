#pragma once
#include <cstdint>

class Communicator {
   public:

	void RecvUpdate(const uint8_t* buf);
    void Send();
    bool is_reply_ = false;
        struct __attribute__((packed)) rxData // ???????
    {
        float vw;
		uint16_t booster_heat_max;
		uint16_t booster_now_heat;
        uint16_t booster_heat_cd;
    };
	
	rxData rx_chassis;
};
extern    uint16_t shooter_limit, shooter_value,shooter_heat;