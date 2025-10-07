#ifndef _KINEMATICS_HPP_
#define _KINEMATICS_HPP_ 

#include "../User/MidLayer/Controllers/signal_processing/Remote_target.hpp"

namespace KinematicsTarget
{
    class kinematictarget
    {
        public:
            void 
            
    }
}

#endif
class kinematics_target : public Kinematics::Wheelset<4>
    {
        public:
            void nofollow(float phi)//0.005512f
            {
                float w, vx, vy;
                w = 8911.0f*dr16_tar.GetRoller();
                vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi*VehicleDirection_w) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi*VehicleDirection_w)); 
                vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi*VehicleDirection_w) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi*VehicleDirection_w)); 
                
                Omini.setVehicleDirection_w(w);
                Omini.setVehicleDirection_vx(vx);
                Omini.setVehicleDirection_vy(vy);
            }

            void follow(float phi)//13.78f
            {
                float w, vx, vy;
                w  = 8911.0f*dr16_tar.GetRoller() /*+ Revolve_PidInstance.NormalPID(M6020_206.theta_planning, 0.0f)*/; 
                vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi));
                vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi));
            
                Omini.setVehicleDirection_w(w);
                Omini.setVehicleDirection_vx(vx);
                Omini.setVehicleDirection_vy(vy);
            }

            void follow_rotate(float phi)//13.78f
            {
                float w, vx, vy;
                w  = 8911.0f*dr16_tar.GetRoller() /*+ Revolve_PidInstance.NormalPID(M6020_206.theta_planning, 0.0f)*/; 
                vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi));
                vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi));
            
                Omini.setVehicleDirection_w(w);
                Omini.setVehicleDirection_vx(vx);
                Omini.setVehicleDirection_vy(vy);            
            }

            void keyboard()
            {
                
            }

    };