#ifndef OMNI_HPP
#define OMNI_HPP

#include "Alg/InverseKinematics/InverseKinematics_Base.hpp"
#include <cmath>

namespace Alg::InverseKinematics 
{
    class Omni : public IKBase 
    {
        public:
            Omni(float r = 1.0f, float s = 1.0f) 
                : R(r), S(s), Motor0(0.0f), Motor1(0.0f), Motor2(0.0f), Motor3(0.0f) {}

            void CalculateVelocities()
            {
                Vx = GetSpeedGain() * (GetSignal_x() *  cosf(GetPhase()) + GetSignal_y() * sinf(GetPhase()));
                Vy = GetSpeedGain() * (GetSignal_x() * -sinf(GetPhase()) - GetSignal_y() * cosf(GetPhase()));
                Vw = GetRotationalGain() * GetSignal_w();
            }

            void SetR(float r) { R = (r > 0.0f) ? r : 1.0f; }  
            void SetS(float s) { S = (s > 0.0f) ? s : 1.0f; }  

            void InvKinematics()
            {
                CalculateVelocities();  
                const float sqrt2_2 = 1.414f / 2.0f;
                
                Motor0 = (-sqrt2_2 * Vx + sqrt2_2 * Vy + Vw*R)/S;
                Motor1 = (-sqrt2_2 * Vx - sqrt2_2 * Vy + Vw*R)/S;
                Motor2 = ( sqrt2_2 * Vx - sqrt2_2 * Vy + Vw*R)/S;
                Motor3 = ( sqrt2_2 * Vx + sqrt2_2 * Vy + Vw*R)/S;
            }

            float GetMotor0() const { return Motor0; }
            float GetMotor1() const { return Motor1; }
            float GetMotor2() const { return Motor2; }
            float GetMotor3() const { return Motor3; }
            

            float GetVx() const { return Vx; }
            float GetVy() const { return Vy; }
            float GetVw() const { return Vw; }

        private:
            float Vx{0.0f};
            float Vy{0.0f};
            float Vw{0.0f};
            float R;
            float S;
            float Motor0;
            float Motor1;
            float Motor2;
            float Motor3;
    };
}

#endif
