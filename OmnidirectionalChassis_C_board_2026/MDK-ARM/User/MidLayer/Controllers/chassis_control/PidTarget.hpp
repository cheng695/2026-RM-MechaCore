#ifndef _PID_TARGET_HPP_
#define _PID_TARGET_HPP_ 

#include "../User/MidLayer/Algorithms/kinematics/chassis_kinematics.hpp"

#define PIDtargetNum 5

extern Kinematic::wheelkinematic ChassisKinematic;

namespace PidTarget
{
    class pidtarget
    {
        public:
            void Setpidtarget()
            {
                ChassisKinematic.KinematicTarget();
                for(uint8_t i = 0; i < 4; i++) //�ĸ����������
                {
                    PidTarget[i] = ChassisKinematic.GetKinematicsTarget(i);
                }
                //PidTarget[4] = ; //���̸�����
            }

            float GetPidTarget(uint8_t idenx)
            {
                return PidTarget[idenx];
            }

        private:
            float PidTarget[PIDtargetNum];        
    };
}

#endif
