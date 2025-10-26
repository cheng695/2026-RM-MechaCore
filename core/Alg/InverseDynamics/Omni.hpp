#ifndef INVERSE_DYNAMICS_OMNI_HPP
#define INVERSE_DYNAMICS_OMNI_HPP

#include "Alg/InverseDynamics/InverseDynamics_Base.hpp"
#include <math.h>

namespace Alg::InverseDynamics
{
    class Omni : public IDBase
    {
        public:
            Omni(float w = 1.0f, float l = 1.0f, float r = 1.0f) 
                : W(w), L(l), R(r)
            {
                sqrt2_4 = sqrt(2.0f) / 4.0f;
                _2sqrt2 = 2.0f * sqrt(2.0f);
                k_inv  = 1.0f / (_2sqrt2 * (L + W)); 

                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = 0.0f;
                }
            }

            void InverseDynamics()
            {
                MotorTorque[0] = (-sqrt2_4 * GetFx() + sqrt2_4 * GetFy() + k_inv * GetTorque()) * R;
                MotorTorque[1] = ( sqrt2_4 * GetFx() + sqrt2_4 * GetFy() - k_inv * GetTorque()) * R;
                MotorTorque[2] = ( sqrt2_4 * GetFx() - sqrt2_4 * GetFy() + k_inv * GetTorque()) * R;
                MotorTorque[3] = (-sqrt2_4 * GetFx() - sqrt2_4 * GetFy() - k_inv * GetTorque()) * R;
            }

            void OmniInvDynamics(float fx, float fy, float torque)
            {
                Set_FxFyTor(fx, fy, torque);
                InverseDynamics();
            }

            float GetF0() const { return MotorTorque[0]; }
            float GetF1() const { return MotorTorque[1]; }
            float GetF2() const { return MotorTorque[2]; }
            float GetF3() const { return MotorTorque[3]; }
        
        private:
            float W; //质心到左右轮
            float L; //质心到前后轮
            float R; //轮子半径
            float MotorTorque[4];
            float sqrt2_4;
            float _2sqrt2;
            float k_inv; 
    };
}

#endif
