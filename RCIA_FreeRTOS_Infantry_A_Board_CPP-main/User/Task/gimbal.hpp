#pragma once
#include <cstdint>
#include "Heat_Detector.hpp"
#define M_PI 3.14159265358979323846
//constexpr float kLowestEuler = -0.349;  // -0.349����
//constexpr float kHighestEuler = 0.61;   // 0.610����
constexpr float kLowestEuler = -1500.0f;
constexpr float kHighestEuler = 600.0f;
extern float yaw_target_euler;
extern float pitch_target_euler;
extern float target_pitch;
extern int16_t friction_target_rpm;
extern int32_t trigger_target_pos;
extern int32_t target_pos;
namespace APP {
    namespace Heat_Detector {

        class Class_ShootFSM {
        public:
            Class_ShootFSM();  // ���캯��
            void Control();
            void HeatLimit();
//				    float  setsum()const
//						{
//						  return Heat_Limit.Current_Detector.sum;
//						
//						}
             ::APP::Heat_Detector::Class_FSM_Heat_Limit Heat_Limit; // ʹ�������ռ��޶�
        private:
            float target_dail_omega;
//            ADRC adrc_Dail_vel; // �����Ѷ���
          
        };

    }  // namespace Heat_Detector
}  

extern APP::Heat_Detector::Class_ShootFSM shoot_fsm;

#ifdef __cplusplus
extern "C" {
#endif

void GimbalTask(void* argument);

#ifdef __cplusplus
}

#endif
