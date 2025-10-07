#ifndef _CHASSIS_CONTROL_HPP_
#define _CHASSIS_CONTROL_HPP_ 

#include <math.h>
#include "../User/MidLayer/Algorithms/algorithm/pid.hpp"
#include "../User/MidLayer/Managers/state_manager/state.hpp"
#include "../User/MidLayer/Controllers/signal_processing/Remote_target.hpp"

extern State::model chass_model;
extern Target::target dr16_tar;
extern motor::GM3508<4> Motor3508;
extern PidTarget::pid_target<PIDQuantity> ChassisPidTarget;
extern PID::PID_ M3508;
extern PID::PID_ M3508_1;
extern PID::PID_ M3508_2;
extern PID::PID_ M3508_3;
extern PID::PID_ M3508_4;
extern PID::PID_ Revolve_PidInstance;

namespace ChassisControl
{
    class modelchoose
    {
        public:
            void ModeSelection();
        private:
    };

    // class kinematics_target : public Kinematics::Wheelset<4>
    // {
    //     public:
    //         void nofollow(float phi)//0.005512f
    //         {
    //             float w, vx, vy;
    //             w = 8911.0f*dr16_tar.GetRoller();
    //             vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi*VehicleDirection_w) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi*VehicleDirection_w)); 
    //             vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi*VehicleDirection_w) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi*VehicleDirection_w)); 
                
    //             Omini.setVehicleDirection_w(w);
    //             Omini.setVehicleDirection_vx(vx);
    //             Omini.setVehicleDirection_vy(vy);
    //         }

    //         void follow(float phi)//13.78f
    //         {
    //             float w, vx, vy;
    //             w  = 8911.0f*dr16_tar.GetRoller() /*+ Revolve_PidInstance.NormalPID(M6020_206.theta_planning, 0.0f)*/; 
    //             vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi));
    //             vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi));
            
    //             Omini.setVehicleDirection_w(w);
    //             Omini.setVehicleDirection_vx(vx);
    //             Omini.setVehicleDirection_vy(vy);
    //         }

    //         void follow_rotate(float phi)//13.78f
    //         {
    //             float w, vx, vy;
    //             w  = 8911.0f*dr16_tar.GetRoller() /*+ Revolve_PidInstance.NormalPID(M6020_206.theta_planning, 0.0f)*/; 
    //             vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi));
    //             vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi));
            
    //             Omini.setVehicleDirection_w(w);
    //             Omini.setVehicleDirection_vx(vx);
    //             Omini.setVehicleDirection_vy(vy);            
    //         }

    //         void keyboard()
    //         {
                
    //         }

    // };

    template<uint8_t N> class motortarget 
    {
        public:
            void Invoke()
            {
                for(uint8_t i = 0; i < N; ++i)
                {
                    ChassisPidTarget.pidset(i);
                    // Pidout[i] = M3508.NormalPID(Motor3508.GetSpeedRpm(i), ChassisPidTarget.GetPidTarget(i));
                }
                Pidout[0] = M3508_1.NormalPID(Motor3508.GetSpeedRef(1), ChassisPidTarget.GetPidTarget(0));
                Pidout[1] = M3508_2.NormalPID(Motor3508.GetSpeedRef(2), ChassisPidTarget.GetPidTarget(1));
                Pidout[2] = M3508_3.NormalPID(Motor3508.GetSpeedRef(3), ChassisPidTarget.GetPidTarget(2));
                Pidout[3] = M3508_4.NormalPID(Motor3508.GetSpeedRef(4), ChassisPidTarget.GetPidTarget(3));
            }

            void Motortarget(uint8_t j)
            {
                for(uint8_t i = 0; i <= j; ++i)
                {
                    MotorTarget[i] = Pidout[i];
                }
            }
            void Omnidirectional()
            {
                for(int i = 0; i < Motor3508.GetMotorCount(); ++i)
                {
                    Invoke();
                    Motortarget(i);
                    Motor3508.SetTarget(i, MotorTarget[i]);
                }
                Motor3508.Send0x200(MotorTarget[0], MotorTarget[1], MotorTarget[2], MotorTarget[3]);
            }    


        private:
            float MotorTarget[N];
            float Pidout[N];
    };
}

#endif
