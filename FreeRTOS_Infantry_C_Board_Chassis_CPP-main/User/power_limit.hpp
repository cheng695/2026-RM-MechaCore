#pragma once
#include "RLS.hpp"
#include "arm_math.h"
#include "variables.hpp"
#define toque_const_3508 0.00036621f
#define rpm_to_rads 0.002908820f

#define M_Pi 3.1415926
#define pMax 120.f
 struct chassis_move_t
{
  Math::RLS<2> rls;
  float speed[4];                  
  float speed_pid[4];
  float motor[4];	
  float  initial_give_power[4];    
  float k2;                         
  float a;                         
  float initial_total_power;       
  float chassis_max_power;          
  float scaled_give_power[4]; 
  float target_speed[4];     // �����Ŀ��ת��
  float speed_error[4];      // ת��������
  float error_weights[4];    // ���Ȩ�ر���
  chassis_move_t() 
        : rls(1e-5f, 0.99999f)  // �������� Math::RLS ��ƥ��
    {
        // ������Ա��ʼ������ѡ��
    }    
  void PowerCare();
	
};

namespace SGPowerControl
{
    struct PowerObj
    {
    public:
		
        float pidOutput;    // torque current command, [-maxOutput, maxOutput], no unit
        float curAv;        // Measured angular velocity, [-maxAv, maxAv], rad/s
        float setAv;        // target angular velocity, [-maxAv, maxAv], rad/s
        float pidMaxOutput; // pid max output
    };
    class power_update
    {
    private:
		
    public:
        Math::RLS<3> rls;
        Matrixf<3, 1> samples;
        Matrixf<3, 1> params;

        int16_t MAXPower;
        power_update()
            : rls(1e-4f, 0.9995f) // ʹ�ù��캯����ʼ���б����г�ʼ��
        {
        }

        /* data */
        float k1 , k2 , k3 = 4.550f;
        float Energy;
        float EstimatedPower;
        float Cur_EstimatedPower;
        float Initial_Est_power[4];
        float EffectivePower;
        float pMaxPower[4];
        double Cmd_MaxT[4];
        void UpRLS(PID *pid, DjiMotor* dji_motor_list, const float toque_const);
        // �ȱ����ŵ������书��
        void UpScaleMaxPow(PID *pid);
        // ����Ӧ���������
        void UpCalcMaxTorque(float *final_Out, DjiMotor* dji_motor_list, PID *pid, const float toque_const);
    };
	class power_ctrl
	{
	public:	
		power_ctrl()
		{
			Wheel_PowerData.MAXPower = 40;
            Wheel_PowerData.k1 =0.982358992;
            Wheel_PowerData.k2 = 3.62282464e-07;
		}
		power_update Wheel_PowerData;
		inline float GetEstWheelPow()
        {
            return Wheel_PowerData.EstimatedPower;
        }
		inline void setMaxPower(float maxPower)
        {
            Wheel_PowerData.MAXPower = maxPower;          
        }
		inline uint16_t getMAXPower()
        {
            return Wheel_PowerData.MAXPower;
        }
	};
}
static inline bool floatEqual(float a, float b) {return fabs(a - b) < 1e-5f;}
extern SGPowerControl::power_ctrl PowerControl;
extern double power_get[4];
//extern chassis_move_t power_care;
#ifdef __cplusplus
extern "C"
{
#endif

    void RlsTask(void *argument);

#ifdef __cplusplus
}
#endif
//#pragma once


// struct chassis_move_t
//{
//  float speed[4];                  
//  float speed_pid[4];               
//  float  initial_give_power[4];    
//  float k2 ;                         
//  float a;                         
//  float initial_total_power;       
//  float chassis_max_power;          
//  float scaled_give_power[4];     
//  void PowerCare();
// 
//};

//extern chassis_move_t power_care;



