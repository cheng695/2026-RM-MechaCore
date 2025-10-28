#ifndef INVERSE_KINEMATICS_OMNI_HPP
#define INVERSE_KINEMATICS_OMNI_HPP

#include "Alg/InverseKinematics/InverseKinematics_Base.hpp"
#include <math.h>

namespace Alg::InverseKinematics 
{
    class Omni : public IKBase 
    {
        public:
            Omni(float r = 1.0f, float s = 1.0f) 
                : R(r), S(s) 
            {
                sqrt2_2 = 1.414f / 2.0f;
                for(int i = 0; i < 4; i++)
                {
                    Motor[i] = 0.0f;
                }
            }

            void CalculateVelocities()
            {
                Vx = GetSpeedGain() * (GetSignal_x() *  cosf(GetPhase()) + GetSignal_y() * sinf(GetPhase()));
                Vy = GetSpeedGain() * (GetSignal_x() * -sinf(GetPhase()) - GetSignal_y() * cosf(GetPhase()));
                Vw = GetRotationalGain() * GetSignal_w();
            }


            void InvKinematics()
            {   
                Motor[0] = (-sqrt2_2 * Vx + sqrt2_2 * Vy + Vw*R)/S;
                Motor[1] = (-sqrt2_2 * Vx - sqrt2_2 * Vy + Vw*R)/S;
                Motor[2] = ( sqrt2_2 * Vx - sqrt2_2 * Vy + Vw*R)/S;
                Motor[3] = ( sqrt2_2 * Vx + sqrt2_2 * Vy + Vw*R)/S;
            }

            void OmniInvKinematics(float vx, float vy, float vw)
            {
                SetSignal_xyw(vx, vy, vw);
                CalculateVelocities(); 
                InvKinematics();
            }

            float GetMotor0() const { return Motor[0]; }
            float GetMotor1() const { return Motor[1]; }
            float GetMotor2() const { return Motor[2]; }
            float GetMotor3() const { return Motor[3]; }
            

            float GetVx() const { return Vx; }
            float GetVy() const { return Vy; }
            float GetVw() const { return Vw; }

        private:
            float Vx{0.0f};
            float Vy{0.0f};
            float Vw{0.0f};
            float R;
            float S;
            float Motor[4];
            float sqrt2_2;
    };
}

#endif
