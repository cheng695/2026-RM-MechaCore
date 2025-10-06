#ifndef _CHASSIS_KINEMATICS_HPP_
#define _CHASSIS_KINEMATICS_HPP_ 

#include "../User/LowLayer/Equipment/motor/djimotor.hpp"

extern motor::GM3508<4> Motor3508;
extern float kinematicsTarget[4];

namespace Kinematics
{
    template<uint8_t N> class Wheelset
    {
        public:
            void Target()
            {
                for(int i = 0; i < N; i++)
                {
                    switch (i)
                    {
                    case 0:
                        kinematicsTarget[i] = VehicleDirection_vx  - VehicleDirection_vy + VehicleDirection_w;
                        break;
                    
                    case 1:
                        kinematicsTarget[i] = -VehicleDirection_vx - VehicleDirection_vy + VehicleDirection_w;
                        break;
                    
                    case 2:
                        kinematicsTarget[i] = -VehicleDirection_vx + VehicleDirection_vy + VehicleDirection_w;
                        break;
                    
                    case 3:
                        kinematicsTarget[i] = VehicleDirection_vx  + VehicleDirection_vy + VehicleDirection_w;

                    default:
                        break;
                    }
                }
            }

            uint8_t GetCount()
            {
                return N;
            }
            
            float GetTarget(uint8_t index)
            {
                return kinematicsTarget[index];
            } 
            
            void setVehicleDirection_w(float w) 
            {
                VehicleDirection_w = w;
            }
            
            void setVehicleDirection_vx(float vx) 
            {
                VehicleDirection_vx = vx;
            }
            
            void setVehicleDirection_vy(float vy) 
            {
                VehicleDirection_vy = vy;
            }

        protected:
            float VehicleDirection_w;
            float VehicleDirection_vx;
            float VehicleDirection_vy;
        
        private:
            float kinematicsTarget[N];
    };
}

#endif
