/**
 * @file power_limit.cpp
 * @author XMX
 * @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ÝµÄ¹ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ã·¨ï¿½ï¿½Î´ï¿½ï¿½ï¿½ï¿½È«ï¿½ï¿½ï¿½Ô£ï¿½
 * @version 1.0
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "power_limit.hpp"
#include "config.hpp"
#include "variables.hpp"
#include "math.h"
#include "chassis.hpp"
#include "cmsis_os2.h"
#include "clamp.hpp"
#include "capacity.hpp"
#include "call_back.hpp"
////chassis_move_t power_care;

float maximum_power;
float sumErr = 0;
float constant = 1;
float effective_power = 0.0f;
float initial_total_power = 0;
float final_Out[4] = {0};
using namespace SGPowerControl;
SGPowerControl::power_ctrl PowerControl;
uint16_t time = 1;

uint8_t sendData[64] ;
void vofaSend(float x1, float x2, float x3, float x4, float x5, float x6)
{
   const uint8_t sendSize = 4;

    *((float *)&sendData[0]) = x1;
    *((float *)&sendData[4]) = x2;
    *((float *)&sendData[8]) = x3;
    *((float *)&sendData[12]) = x4;
    *((uint32_t *)&sendData[sizeof(float) * 7]) = 0x7f800000;
	
   	HAL_UART_Transmit_DMA(&huart6, sendData, sizeof(float) * 8);
}

void RlsTask(void *argument)
{
	for(;;)
	{
		PowerControl.Wheel_PowerData.UpRLS(pid_vel[0],dji_motor_list[0],toque_const_3508);
		PowerControl.Wheel_PowerData.UpScaleMaxPow(pid_vel[0]);
		PowerControl.Wheel_PowerData.UpCalcMaxTorque(final_Out, dji_motor_list[0], pid_vel[0], toque_const_3508);

//		vofaSend(PowerControl.Wheel_PowerData.Cur_EstimatedPower,
//					PowerControl.Wheel_PowerData.EstimatedPower,
//					capacity.target_power,
//					0,0,0);

		osDelay(time);
	}
}

void power_update::UpRLS(PID *pid_vel, DjiMotor* dji_motor_list, const float toque_const)
{
	float realtime_power = capacity.target_power;     //ï¿½ï¿½ï¿½ç·´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½

	EffectivePower = 0;  //ï¿½ï¿½Ð§ï¿½ï¿½ï¿½ï¿½,ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	samples[0][0] = 0;
	samples[1][0] = 0;

	for(int i = 0; i < 4; i++)
	{
		EffectivePower += dji_motor_list[i].actual_current_ * dji_motor_list[i].actual_rpm_ * toque_const * rpm_to_rads;

		samples[0][0] +=  fabs(dji_motor_list[i].actual_rpm_ * rpm_to_rads) ;
		samples[1][0] +=  dji_motor_list[i].actual_current_ * dji_motor_list[i].actual_current_ * toque_const * toque_const;
	}

	params = rls.update(samples, realtime_power - EffectivePower);
	k1 = fmaxf(params[0][0], 1e-5f); // ï¿½ï¿½Ö¹ï¿½ï¿½Öµ
	k2 = fmaxf(params[1][0], 1e-5f);
	
		Cur_EstimatedPower = k1 * samples[0][0] + k2 * samples[1][0] +  EffectivePower + k3;
		EstimatedPower = 0;  //Ô¤ï¿½â¹¦ï¿½ï¿½

		for(int i = 0; i < 4; i++)
		{
			Initial_Est_power[i] = pid_vel[i].output_ * toque_const * dji_motor_list[i].actual_rpm_ * rpm_to_rads +
									fabs( dji_motor_list[i].actual_rpm_ * rpm_to_rads) * k1 +
									pid_vel[i].output_ * toque_const * pid_vel[i].output_ * toque_const * k2 + k3/4.0f;
			if(Initial_Est_power[i] < 0)
				continue;	
			EstimatedPower += Initial_Est_power[i];
		}	
}	

void power_update::UpScaleMaxPow(PID *pid_vel)
{
	sumErr = 0;
    for (int i = 0; i < 4; i++) {
        sumErr += fabsf(pid_vel[i].error_);
    }
  
    for (int i = 0; i < 4; i++) {
        pMaxPower[i] = MAXPower * (fabsf(pid_vel[i].error_) / sumErr);
        if (pMaxPower[i] < 0) {
            continue;
        }
    }
}


double power_get[4];
void power_update::UpCalcMaxTorque(float *final_Out, DjiMotor* dji_motor_list, PID *pid_vel, const float toque_const)
{
    if (EstimatedPower > MAXPower) {
        for (int i = 0; i < 4; i++) {
            float rpm = dji_motor_list[i].actual_rpm_ * rpm_to_rads;

            float A = k2;
            float B = rpm;
            float C = k1 * fabs(rpm) + k3 / 4.0f - pMaxPower[i];

            float delta = (B * B) - 4.0f * A * C;
			if (delta < 0) {
				Cmd_MaxT[i] = (-B) / (2.0f * A); // ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð§sqrt
			}
			else {
				Cmd_MaxT[i] = pid_vel[i].output_ > 0.0f ? (-B + sqrtf(delta)) / (2.0f * A) / toque_const
														: (-B - sqrtf(delta)) / (2.0f * A) / toque_const;

				Cmd_MaxT[i] = clamp(Cmd_MaxT[i], -16384.0f, 16384.0f);

				power_get[i] = Cmd_MaxT[i];
			}
			
        }
    }
   else{
		    power_get[0]=pid_vel_201.output_;
		    power_get[1]=pid_vel_202.output_;
		    power_get[2]=pid_vel_203.output_;
		    power_get[3]=pid_vel_204.output_;
		
	    
	}
}

//chassis_move_t power_care;       
//float maximum_power;

///// @brief ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
//void  chassis_move_t::PowerCare() {
//    //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
//	
////    auto i1 = motor_201.actual_current_;
////    auto i2 = motor_202.actual_current_;
////    auto i3 = motor_203.actual_current_;
////    auto i4 = motor_204.actual_current_;

////    if (referee.power_heat_data.buffer_energy > kWarningPowerBuffer) {
////        //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
////        maximum_power = (referee.power_heat_data.buffer_energy - kWarningPowerBuffer) / 0.02f;
////    } else {
////        //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
////        maximum_power = referee.robot_status.chassis_power_limit;
////    }

////    //ï¿½ï¿½ï¿½ï¿½ï¿½Üµï¿½ï¿½ï¿½
////    auto chassis_motor_current = i1 + i2 + i3 + i4;

////    //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿?
////    if (chassis_motor_current / 819.2 * kDefaultBatteryVoltage > maximum_power) {
////        motor_201.input_ = i1 / chassis_motor_current * (maximum_power / kDefaultBatteryVoltage);
////        motor_202.input_ = i2 / chassis_motor_current * (maximum_power / kDefaultBatteryVoltage);
////        motor_203.input_ = i3 / chassis_motor_current * (maximum_power / kDefaultBatteryVoltage);
////        motor_204.input_ = i4 / chassis_motor_current * (maximum_power / kDefaultBatteryVoltage);
////    }


//	speed_pid[0]=motor_201.input_;
//	speed_pid[1]=motor_202.input_;
//	speed_pid[2]=motor_203.input_;
//	speed_pid[3]=motor_204.input_;
//	speed[0]=motor_201.actual_rpm_;
//	speed[1]=motor_202.actual_rpm_;
//	speed[2]=motor_203.actual_rpm_;
//	speed[3]=motor_204.actual_rpm_;
//	float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55   ï¿½Oï¿½xï¿½`?
//	  float constant = 1.3; //ï¿½ï¿½?ï¿½\ï¿½vï¿½ï¿½ï¿½î???ï¿½ï¿½ï¿½^??           
//		// get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
//		// PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 30);
//		// get_chassis_max_power(&max_power_limit);
//		// input_power = max_power_limit - chassis_power_control->buffer_pid.out; // Input power floating at maximum power
//	
//		// CAN_CMD_CAP(input_power); // set the input power of capacitor controller
//	
//		// if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
//		// {
//		// 	cap_state = 0;
//		// }
//		// if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
//		// {
//		// 	cap_state = 1;
//		// }
//	
//		// if (cap_measure.cap_percent > 5)
//		// {
//		// 	if (cap_state == 0)
//		// 	{
//		// 		chassis_max_power = input_power + 5; //  
//		// 	}
//		// 	else
//		// 	{
//		// 		chassis_max_power = input_power + 200;
//		// 	}
//		// }
//		// else
//		// {
//		// 	chassis_max_power = input_power;
//		// }
//	
//		for (uint8_t i = 0; i < 4; i++) //ï¿½`?ï¿½P?ï¿½n?ï¿½ï¿½ï¿½\ï¿½v
//		{
//		                 //??ï¿½ï¿½pid?ï¿½J?ï¿½ï¿½
//			initial_give_power[i] = speed_pid[i] * toque_coefficient *speed[i] +
//			k2 * speed[i] * speed[i] +
//			a * speed_pid[i] * speed_pid[i] + constant;//?ï¿½ï¿½\ï¿½v

//			if (initial_give_power[i]<0) continue;//ï¿½P??ï¿½wï¿½ï¿½ï¿½\ï¿½vï¿½Oï¿½_ï¿½ï¿½ï¿½_0
//				initial_total_power = initial_give_power[0]+initial_give_power[1]+initial_give_power[2]+initial_give_power[3]; //?ï¿½\ï¿½v
//		}
//	
//		if (initial_total_power > chassis_max_power) //ï¿½P??ï¿½\ï¿½vï¿½Oï¿½ï¿½ï¿½Oï¿½jï¿½_ï¿½Ì?jï¿½\ï¿½v 
//		{
//			float power_scale = chassis_max_power /initial_total_power;//ï¿½\ï¿½vï¿½ï¿½ï¿? ?ï¿½Ì?jï¿½\ï¿½v?ï¿½\ï¿½vï¿½ï¿½ï¿½ï¿½ï¿??ï¿½t
//			for (uint8_t i = 0; i < 4; i++)   
//			{
//				scaled_give_power[i] = initial_give_power[i] * power_scale; // ?ï¿½w?ï¿½Eï¿½ï¿½ï¿½\ï¿½v=?ï¿½wï¿½\ï¿½v*ï¿½\ï¿½vï¿½ï¿½ï¿?
//				if (scaled_give_power[i] < 0)
//				{
//					continue;                                          //ï¿½P??ï¿½w?ï¿½Eï¿½\ï¿½vï¿½Oï¿½_ï¿½pï¿½_0
//				}
//	
//				float b = toque_coefficient * speed[i];          //b=ï¿½Oï¿½x*ï¿½tï¿½×¤ï¿½?
//				float c = a * speed[i] * speed[i] - scaled_give_power[i] + constant;
//				float inside = b * b - 4 * a * c;
//	
//				if (inside < 0)
//				{
//					continue;
//				}
//				else if (speed_pid[i] > 0) // Selection of the calculation formula according to the direction of the original moment
//				{
//					float temp = (-b + sqrt(inside)) / (2 * a);
//					if (temp > 16000)
//					{
//						speed_pid[i] = 16000;
//					}
//					else
//					speed_pid[i] = temp;
//				}
//				else
//				{
//					float temp = (-b - sqrt(inside)) / (2 * a);
//					if (temp < -16000)
//					{
//						speed_pid[i] = -16000;
//					}
//					else
//					speed_pid[i] = temp;
//				}
//			}
//		}
//	}
