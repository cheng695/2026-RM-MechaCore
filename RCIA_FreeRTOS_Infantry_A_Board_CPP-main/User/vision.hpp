#pragma once
#include <cstdint>

enum AimType {
    kArmor,
    kRobotHub,
};
struct vision_t
{
	uint8_t vision_TX_data[18];
	uint8_t vision_RX_data[18];
	float vision_yaw;
	float vision_pitah;
	int16_t pitch_360;
	int32_t yaw_360;
	uint8_t vision_ready;
	uint8_t vision_fire;
	bool vision_bool;
	
};
class Vision {
   public:
    bool is_use_ = false;
    bool is_aimed_ = false;
    bool is_reply = false;
    enum AimType aim_type_;

    float origin_yaw_ = 0;
    float origin_pitch_ = 0;
    float origin_yaw_hub_ = 0;
    float origin_pitch_hub = 0;

    float yaw_increment_temp = 0;
    float pitch_increment_temp = 0;
    float yaw_hub_increment_temp = 0;
    float pitch_hub_increment_temp = 0;

    float yaw_increament = 0;
    float pitch_increment = 0;
    float yaw_hub_increment = 0;
    float pitch_hub_increment = 0;

    uint16_t fire_flag_temp = 0;
    bool fire_flag = false;
    bool fire_latch = false;

    uint32_t recv_time = 0;

    void RecvUpdate(const uint8_t* buf);
    void Send();
   private:
    bool AbsoluteFilte() const;
};

extern float int_time;

extern uint8_t vision_flag;