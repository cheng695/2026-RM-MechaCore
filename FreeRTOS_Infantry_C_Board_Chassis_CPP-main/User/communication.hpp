#pragma once
#include <cstdint>

enum AimType {
    kArmor,
    kRobotHub,
};

class Communicator {
   public:
    float theta = 0;
    bool vision_is_use = false;
    AimType vision_aim_type;
    bool friction_is_enable;
    uint8_t vision_is_aimed;
	float pitch_;
	void RecvUpdate(const uint8_t* buf);
    void Send();
	bool is_reply_ = false;
   
    struct __attribute__((packed)) txData // ÔÆÌ¨Êý¾Ý
    {
        float vw;
		uint16_t booster_heat_max;
		uint16_t booster_now_heat;
        uint16_t booster_heat_cd;
    };
	
	txData tx_gimbal;
	uint8_t tx_buf[20];
};
