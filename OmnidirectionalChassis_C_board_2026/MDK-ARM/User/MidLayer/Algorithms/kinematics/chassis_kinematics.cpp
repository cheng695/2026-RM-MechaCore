#include "chassis_kinematics.hpp"

uint8_t kinematicsTarget[4] = {1, 2, 3, 4};
 
Kinematics::Chassis<4> Omini(kinematicsTarget);

void Kinematics::Chassis<4>::Target()
{
    kinematicsTarget[0] = VehicleDirection_vx  - VehicleDirection_vy + VehicleDirection_w;
    kinematicsTarget[1] = -VehicleDirection_vx - VehicleDirection_vy + VehicleDirection_w;
    kinematicsTarget[2] = -VehicleDirection_vx + VehicleDirection_vy + VehicleDirection_w;
    kinematicsTarget[3] = VehicleDirection_vx  + VehicleDirection_vy + VehicleDirection_w;
}

void Kinematics::Chassis<4>::Omnidirectional()
{
    Target();
    for(int i = 0; i < Motor3508.GetMotorCount(); i++)
    {
        Motor3508.SetTarget(i, kinematicsTarget[i]);
    }
}    
