#include "InHpp.hpp"
/*  =========================== 全局变量的初始化 ===========================  */ 
MyRefereeSystemData_t MyRefereeSystemData = {0};//判断裁判系统断连
robot_status_t ext_robot_status_t_0x0201 = {0};//0x0201 机器人状态数据
power_heat_data_t ext_power_heat_data_0x0202 = {0};//0x0202 实时功率热量数据
shoot_data_t shoot_data_0x0207 = {0}; //飞机射速
game_status_t game_status_0x001 = {0};

//飞镖发射口倒计时
ext_dart_remaining_time_t ext_dart_remaining_time_0x0105 = { 0 };
//飞镖机器人客户端指令数据
ext_dart_client_cmd_t ext_dart_client_cmd_0x020A = { 0 };

unsigned char MyRefereeSys8Data = 0;


/*  =========================== 函数的声明 ===========================  */ 
void MyRefereeSystemParseData(uint8_t* MypDatas,int size);
void MyRefereeSystemGetData(uint8_t MypData);
void MyRefereeSystemParse();


/************************************************************************
* @brief:      	void
* @param[in]: 	void
* @retval:      void
* @details:    	void
*************************************************************************/
void MyRefereeSystemParseData(uint8_t* MypDatas,int size)
{
	MyRefereeSystemData.SOF = MypDatas[0];
	MyRefereeSystemData.data_length = MypDatas[1] | MypDatas[2] << 8;
	MyRefereeSystemData.seq = MypDatas[3];
	MyRefereeSystemData.CRC8 = MypDatas[4];
	MyRefereeSystemData.cmd_id = MypDatas[5] | MypDatas[6] << 8;
	if(MyRefereeSystemData.data_length > 50)
		return;
	for(uint16_t i = 0;i < MyRefereeSystemData.data_length;i++)
	{
		MyRefereeSystemData.data[i] = MypDatas[7 + i];
	}
	MyRefereeSystemData.frame_tail = MypDatas[7 + MyRefereeSystemData.data_length];
	switch (MyRefereeSystemData.cmd_id)
	{
	case 0x001:
	memcpy(&game_status_0x001,(void*)MyRefereeSystemData.data,sizeof(game_status_0x001));
		break;
	case 0x0201:
		memcpy(&ext_robot_status_t_0x0201,(void*)MyRefereeSystemData.data,sizeof(ext_robot_status_t_0x0201));
		break;	
	case 0x0202:
		memcpy(&ext_power_heat_data_0x0202,(void*)MyRefereeSystemData.data,sizeof(ext_power_heat_data_0x0202));
		break;	
	case 0x0207:
		memcpy(&shoot_data_0x0207,(void*)MyRefereeSystemData.data,sizeof(shoot_data_0x0207));
	case 0x0105:
		memcpy(&ext_dart_remaining_time_0x0105,(void*)MyRefereeSystemData.data,sizeof(ext_dart_remaining_time_0x0105));
		break;
	case 0x020A:
		memcpy(&ext_dart_client_cmd_0x020A,(void*)MyRefereeSystemData.data,sizeof(ext_dart_client_cmd_0x020A));
		break;
	default:
		break;
	}
}


/************************************************************************
* @brief:      	void
* @param[in]: 	void
* @retval:      void
* @details:    	void
*************************************************************************/
void MyRefereeSystemGetData(uint8_t MypData)
{
	static int idx = 0,MypDataSize = 0;
	static uint8_t MypDataS[50] = { 0 };
	if(MypData == 0xA5)
	{
			MypDataSize++;
	}
	if(MypDataSize == 2)
	{
		MyRefereeSystemParseData(MypDataS,idx);
		MypDataSize = 1;
		idx = 0;
	}
	if(MypDataSize == 1)
	{
		MypDataS[idx++] = MypData;
	}
}


/************************************************************************
* @brief:      	void
* @param[in]: 	void
* @retval:      void
* @details:    	void
*************************************************************************/
void MyRefereeSystemParse()
{
	MyRefereeSystemGetData(MyRefereeSys8Data);
	HAL_UART_Receive_IT(&HuartHandle_RMRefereeSystem,&MyRefereeSys8Data,sizeof(MyRefereeSys8Data));
	//Report.RMRefereeSystemCom.Count_Paparazzi++;
}
