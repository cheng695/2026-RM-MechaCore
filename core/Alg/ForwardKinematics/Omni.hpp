#ifndef FORWARD_KINEMATICS_OMNI_HPP
#define FORWARD_KINEMATICS_OMNI_HPP

#include "Alg/ForwardKinematics/ForwardKinematics_Base.hpp"
#include <cmath>

namespace Alg::ForwardKinematics
{
    class Omni : public FKBase
    {
        public:
            Omni(float r = 1.0f, float s = 1.0f) 
                : R(r), S(s), ChassisVx(0.0f), ChassisVy(0.0f), ChassisVw(0.0f) 
            {
                sqrt2_S_over_4 = 1.41421356237f * S / 4.0f;
                S_over_4R = S / (4.0f * R);
            }

            void ForKinematics()
            {
                ChassisVx = (-Get_w0() - Get_w1() + Get_w2() + Get_w3()) * sqrt2_S_over_4;
                ChassisVy = ( Get_w0() - Get_w1() - Get_w2() + Get_w3()) * sqrt2_S_over_4;
                ChassisVw = ( Get_w0() + Get_w1() + Get_w2() + Get_w3()) * S_over_4R ;
            }
            
            void OmniForKinematics(float w0, float w1, float w2, float w3)
            {
                Set_w0w1w2w3(w0, w1, w2, w3);
                ForKinematics();
            }

            float GetRadius() const { return R; }
            float GetScaling() const { return S; }
            float GetChassisVx() const { return ChassisVx; }
            float GetChassisVy() const { return ChassisVy; }
            float GetChassisVw() const { return ChassisVw; }

        private:
            float R;
            float S;
            float ChassisVx;
            float ChassisVy;
            float ChassisVw;
            float sqrt2_S_over_4;
            float S_over_4R;
    };
} 

#endif
