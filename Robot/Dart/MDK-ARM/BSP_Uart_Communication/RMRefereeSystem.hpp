#ifndef __RMRefereeSystem_Hpp__
#define __RMRefereeSystem_Hpp__
#include "main.h"

#define HuartHandle_RMRefereeSystem huart6
#define HuartDistance_RMRefereeSystem USART6

extern unsigned char MyRefereeSys8Data;

typedef __packed struct 
{ 
  uint8_t game_type : 4; 
  uint8_t game_progress : 4; 
  uint16_t stage_remain_time; 
  uint64_t SyncTimeStamp; 
}game_status_t; 
extern game_status_t game_status_0x001;

typedef __packed struct 
{
	uint8_t SOF;//数据帧起始字节，固定值为 0xA5
	uint16_t data_length;//数据帧中 data 的长度
	uint8_t seq;//包序号
	uint8_t CRC8;//帧头 CRC8 校验
	uint16_t cmd_id;//命令码 ID
	uint8_t data[6+105];//数据
	uint16_t frame_tail;//CRC16，整包校验
}MyRefereeSystemData_t;
extern MyRefereeSystemData_t MyRefereeSystemData;

typedef __packed struct//0x0201 机器人性能体系数据，固定以10Hz 频率发送
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t current_HP; 
uint16_t maximum_HP;
uint16_t shooter_barrel_cooling_value;
uint16_t shooter_barrel_heat_limit;
uint16_t chassis_power_limit; 
uint8_t power_management_gimbal_output : 1;
uint8_t power_management_chassis_output : 1; 
uint8_t power_management_shooter_output : 1;
}robot_status_t;
extern robot_status_t ext_robot_status_t_0x0201;

typedef __packed struct//0x0202 机器人底盘功率和枪口热量数据
{ 
  uint16_t chassis_voltage; 
  uint16_t chassis_current; 
  float chassis_power; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
}power_heat_data_t; 
extern power_heat_data_t ext_power_heat_data_0x0202;

typedef __packed struct//0x0207 机器人射击数据
{ 
  uint8_t bullet_type;  
  uint8_t shooter_number; 
  uint8_t launching_frequency;  
  float initial_speed;  
}shoot_data_t;
extern shoot_data_t shoot_data_0x0207;

//飞镖机器人客户端指令数据   0x020A
typedef __packed struct
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;
extern ext_dart_client_cmd_t ext_dart_client_cmd_0x020A;

//飞镖发射口倒计时     0x0105
typedef __packed struct
{
 uint8_t dart_remaining_time;
 uint16_t dart_info; 
} ext_dart_remaining_time_t;
extern ext_dart_remaining_time_t ext_dart_remaining_time_0x0105;

void MyRefereeSystemParse(void);
#endif
