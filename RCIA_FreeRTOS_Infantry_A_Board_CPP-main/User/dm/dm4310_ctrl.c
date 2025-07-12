#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
#include "string.h"

motor_t motor[Motor1];

int8_t motor_id = 0x03;

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310�����ʼ������	
* @param:      	void
* @retval:     	void
* @details:    	��ʼ������DM4310�ͺŵĵ��������Ĭ�ϲ����Ϳ���ģʽ��
*               �ֱ��ʼ��Motor1��Motor2������ID������ģʽ������ģʽ����Ϣ��
************************************************************************
**/
void dm4310_motor_init(void)
{
	// ��ʼ��Motor1��Motor2�ĵ���ṹ
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));

	// ����Motor1�ĵ����Ϣ
	motor[Motor1].id = 0x3;
	motor[Motor1].ctrl.mode = 0;		// 0: MITģʽ   1: λ���ٶ�ģʽ   2: �ٶ�ģʽ
	motor[Motor1].cmd.mode = 0;
	
	motor->cmd.kd_set  = 0;
	motor->cmd.kp_set  = 0;
	motor->cmd.pos_set = 0;
	motor->cmd.vel_set = 0;
	motor->cmd.tor_set = 0;
	
	dm4310_set(&motor[Motor1]);
}
/**
************************************************************************
* @brief:      	motor_para_add: �޸ĵ����������
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬�������������Ϣ
* @retval:     	void
* @details:    	���ݵ�ǰLCD��־λ��lcd_flag�����޸�ָ������Ĳ�����Ϣ��
*               ����lcd_flag�Ĳ�ͬ�����޸ĵ��ID������ģʽ��λ���趨���ٶ��趨��
*               Ť���趨�����������΢������Ȳ�����
************************************************************************
**/
// void motor_para_add(motor_t *motor)
// {
// 	switch(lcd_flag)
// 	{
// 		case 0:
// 			motor_id += 1;
// 			if (motor_id > num)
// 				motor_id = 1;
// 			break;
// 		case 1:
// 			motor->ctrl.mode += 1;
// 			if (motor->ctrl.mode > 2)
// 				motor->ctrl.mode = 0;
// 			break;
// 		case 2:
// 			motor->cmd.pos_set += 1.0f;
// 			break;
// 		case 3:
// 			motor->cmd.vel_set += 1.0f;
// 			break;
// 		case 4:
// 			motor->cmd.tor_set += 1.0f;
// 			break;
// 		case 5:
// 			motor->cmd.kp_set += 1.0f;
// 			break;
// 		case 6:
// 			motor->cmd.kd_set += 0.5f;
// 			break;
// 	}
// }
/**
************************************************************************
* @brief:      	motor_para_minus: ��С�����������
* @param[in]:   motor:   ָ��motor_t�ṹ��ָ�룬�������������Ϣ
* @retval:     	void
* @details:    	���ݵ�ǰLCD��־λ��lcd_flag������Сָ������Ĳ�����Ϣ��
*               ����lcd_flag�Ĳ�ͬ���ɼ�С���ID������ģʽ��λ���趨���ٶ��趨��
*               Ť���趨�����������΢������Ȳ�����
************************************************************************
**/
// void motor_para_minus(motor_t *motor)
// {
// 	switch(lcd_flag)
// 	{
// 		case 0:
// 			motor_id -= 1;
// 			if (motor_id < 1)
// 				motor_id = num;
// 			break;
// 		case 1:
// 			motor->ctrl.mode -= 1;
// 			if (motor->ctrl.mode < 0)
// 				motor->ctrl.mode = 2;
// 			break;
// 		case 2:
// 			motor->cmd.pos_set -= 1.0f;
// 			break;
// 		case 3:
// 			motor->cmd.vel_set -= 1.0f;
// 			break;
// 		case 4:
// 			motor->cmd.tor_set -= 1.0f;
// 			break;
// 		case 5:
// 			motor->cmd.kp_set -= 1.0f;
// 			break;
// 		case 6:
// 			motor->cmd.kd_set -= 0.5f;
// 			break;
// 	}
// }
/**
************************************************************************
* @brief:      	ctrl_enable: ���õ�����ƺ���
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ö�Ӧ�ĵ�����ơ�
*               ����ָ�������������־��������dm4310_enable�������õ����
************************************************************************
**/
void ctrl_enable(void)
{
	switch(motor_id)
	{
		case 3:
			// ����Motor1�ĵ������
			motor[Motor1].start_flag = 1;
			dm4310_enable(&hcan2, &motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_disable: ���õ�����ƺ���
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ö�Ӧ�ĵ�����ơ�
*               ����ָ�������������־Ϊ0��������dm4310_disable�������õ����
************************************************************************
**/
void ctrl_disable(void)
{
	switch(motor_id)
	{
		case 3:
			// ����Motor1�ĵ������
			motor[Motor1].start_flag = 0;
			dm4310_disable(&hcan2, &motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_set: ���õ����������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ö�Ӧ����Ĳ�����
*               ����dm4310_set��������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_set(void)
{
	switch(motor_id)
	{
		case 3:
			// ����Motor1�ĵ������
			dm4310_set(&motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_para: ��������������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id���������Ӧ����Ĳ�����
*               ����dm4310_clear�������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_clear_para(void)
{
	switch(motor_id)
	{
		case 1:
			// ���Motor1�ĵ������
			dm4310_clear_para(&motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_err: ������������Ϣ
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id���������Ӧ����Ĳ�����
*               ����dm4310_clear�������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_clear_err(void)
{
	switch(motor_id)
	{
		case 1:
			// ���Motor1�ĵ���������
			dm4310_clear_err(&hcan1, &motor[Motor1]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_add: ���ӵ����������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������Ӷ�Ӧ����Ĳ�����
*               ����motor_para_add��������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
// void ctrl_add(void)
// {
// 	switch(motor_id)
// 	{
// 		case 1:
// 			// ����Motor1�ĵ������
// 			motor_para_add(&motor[Motor1]);
// 			break;
// 	}
// }
/**
************************************************************************
* @brief:      	ctrl_minus: ���ٵ����������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������ٶ�Ӧ����Ĳ�����
*               ����motor_para_minus��������ָ������Ĳ���������Ӧ�ⲿ���
************************************************************************
**/
// void ctrl_minus(void)
// {
// 	switch(motor_id)
// 	{
// 		case 1:
// 			// ����Motor1�ĵ������
// 			motor_para_minus(&motor[Motor1]);
// 			break;
// 	}
// }
/**
************************************************************************
* @brief:      	ctrl_send: ���͵�����������
* @param:      	void
* @retval:     	void
* @details:    	���ݵ�ǰ���ID��motor_id�������Ӧ������Ϳ������
*               ����dm4310_ctrl_send������ָ��������Ϳ����������Ӧ�ⲿ���
************************************************************************
**/
void ctrl_send(void)
{
			 // ��Motor1���Ϳ�������
	dm4310_ctrl_send(&hcan2, &motor[Motor1]);
}

