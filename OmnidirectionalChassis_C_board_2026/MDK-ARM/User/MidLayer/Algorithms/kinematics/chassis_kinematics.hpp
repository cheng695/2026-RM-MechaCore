#ifndef _CHASSIS_KINEMATICS_HPP_
#define _CHASSIS_KINEMATICS_HPP_ 

#include <math.h>

extern float kinematicsTarget[4];

namespace Kinematic
{
    class wheelkinematic
    {
        enum model_wheel
            {
                Omni,
                Mecanum,
                steering
            };
        public:
            void KinematicTarget()
            {
                model_wheel wheel_choose = Omni;
                switch(wheel_choose)
                {
                    case Omni:
                        kinematicsTarget[0] = VehicleDirection_vx  - VehicleDirection_vy + VehicleDirection_w;
                        kinematicsTarget[1] = -VehicleDirection_vx - VehicleDirection_vy + VehicleDirection_w;
                        kinematicsTarget[2] = -VehicleDirection_vx + VehicleDirection_vy + VehicleDirection_w;
                        kinematicsTarget[3] = VehicleDirection_vx  + VehicleDirection_vy + VehicleDirection_w;
                        break;
                    case Mecanum:
                        kinematicsTarget[0] = VehicleDirection_vx  - VehicleDirection_vy + VehicleDirection_w;
                        kinematicsTarget[1] = -VehicleDirection_vx - VehicleDirection_vy + VehicleDirection_w;
                        kinematicsTarget[2] = -VehicleDirection_vx + VehicleDirection_vy + VehicleDirection_w;
                        kinematicsTarget[3] = VehicleDirection_vx  + VehicleDirection_vy + VehicleDirection_w;
                        break;
                    case steering: 
                        const float cos45 = 0.70710678118f; // cos(45бу)
                        const float sin45 = 0.70710678118f; // sin(45бу)

                        kinematicsTarget[0] = sqrtf((VehicleDirection_vy - VehicleDirection_w * cos45)*(VehicleDirection_vy - VehicleDirection_w * cos45) + (VehicleDirection_vx - VehicleDirection_w * sin45)*(VehicleDirection_vx - VehicleDirection_w * sin45));
                        kinematicsTarget[1] = sqrtf((VehicleDirection_vy - VehicleDirection_w * cos45)*(VehicleDirection_vy - VehicleDirection_w * cos45) + (VehicleDirection_vx + VehicleDirection_w * sin45)*(VehicleDirection_vx + VehicleDirection_w * sin45));
                        kinematicsTarget[2] = sqrtf((VehicleDirection_vy + VehicleDirection_w * cos45)*(VehicleDirection_vy + VehicleDirection_w * cos45) + (VehicleDirection_vx + VehicleDirection_w * sin45)*(VehicleDirection_vx + VehicleDirection_w * sin45));
                        kinematicsTarget[3] = sqrtf((VehicleDirection_vy + VehicleDirection_w * cos45)*(VehicleDirection_vy + VehicleDirection_w * cos45) + (VehicleDirection_vx - VehicleDirection_w * sin45)*(VehicleDirection_vx - VehicleDirection_w * sin45));

                        wheel_theta[0] = atan2f(VehicleDirection_vy - VehicleDirection_w * cos45, VehicleDirection_vx - VehicleDirection_w * sin45);
                        wheel_theta[1] = atan2f(VehicleDirection_vy - VehicleDirection_w * cos45, VehicleDirection_vx + VehicleDirection_w * sin45);
                        wheel_theta[2] = atan2f(VehicleDirection_vy + VehicleDirection_w * cos45, VehicleDirection_vx + VehicleDirection_w * sin45);
                        wheel_theta[3] = atan2f(VehicleDirection_vy + VehicleDirection_w * cos45, VehicleDirection_vx - VehicleDirection_w * sin45);
                        
                        break;
                }
            }

            void SetVehicleDirection_w(float w) 
            {
                VehicleDirection_w = w;
            }
            
            void SetVehicleDirection_vx(float vx) 
            {
                VehicleDirection_vx = vx;
            }
            
            void SetVehicleDirection_vy(float vy) 
            {
                VehicleDirection_vy = vy;
            }

            float GetVehicleDirection_w() const
            {
                return VehicleDirection_w;
            }

            float GetVehicleDirection_vx() const
            {
                return VehicleDirection_vx;
            }

            float GetVehicleDirection_vy() const
            {
                return VehicleDirection_vy;
            }

            float GetKinematicsTarget(uint8_t i)
            {
                return kinematicsTarget[i];
            }

            float GetWheelTheta(uint8_t i)
            {
                return wheel_theta[i];
            }

        protected:
            float VehicleDirection_w;
            float VehicleDirection_vx;
            float VehicleDirection_vy;
            float kinematicsTarget[4];
            float wheel_theta[4];
    };
}

#endif
