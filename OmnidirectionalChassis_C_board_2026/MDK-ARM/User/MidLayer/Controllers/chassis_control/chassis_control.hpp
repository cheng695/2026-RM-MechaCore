#ifndef _CHASSIS_CONTROL_HPP_
#define _CHASSIS_CONTROL_HPP_ 

#include <math.h>
#include "../User/MidLayer/Controllers/signal_processing/Remote_target.hpp"
#include "../User/MidLayer/Algorithms/kinematics/chassis_kinematics.hpp"
#include "../User/MidLayer/Controllers/chassis_control/PidTarget.hpp"
#include "../User/MidLayer/Algorithms/algorithm/pid.hpp"
#include "../User/LowLayer/Equipment/motor/djimotor.hpp"

extern RemoteTarget::remotetarget dr16_tar;
extern Kinematic::wheelkinematic ChassisKinematic;
extern PidTarget::pidtarget chassis_pidtarget;
extern PID::PID_ M3508[4], Revolve_PidInstance;
extern motor::GM3508<4> Motor3508;

namespace ChassisControl
{
    class modelchoose
    {
        public:
            void ModeSelection();

            void TransmitSignal()
            {
                ChassisKinematic.SetVehicleDirection_vx(this->vx);
                ChassisKinematic.SetVehicleDirection_vy(this->vy);
                ChassisKinematic.SetVehicleDirection_w(this->w);
            }

            void nofollow(float phi)
            {
                w = 8911.0f*dr16_tar.GetRoller();
                vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi*ChassisKinematic.GetVehicleDirection_w()) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi*ChassisKinematic.GetVehicleDirection_w())); 
                vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi*ChassisKinematic.GetVehicleDirection_w()) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi*ChassisKinematic.GetVehicleDirection_w())); 
                TransmitSignal();
            }

            void follow(float phi)
            {
                w  = 8911.0f*dr16_tar.GetRoller() /*+ Revolve_PidInstance.NormalPID(M6020_206.theta_planning, 0.0f)*/; 
                vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi));
                vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi));
                TransmitSignal();
            }

            void rotate_follow(float phi)
            {
                w  = 8911.0f*dr16_tar.GetRoller() /*+ Revolve_PidInstance.NormalPID(M6020_206.theta_planning, 0.0f)*/; 
                vx = 8911.0f*(dr16_tar.Getvx_left() *  cosf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * sinf(/*M6020_206.theta_planning*/ + 0.02f*phi));
                vy = 8911.0f*(dr16_tar.Getvx_left() * -sinf(/*M6020_206.theta_planning*/ + 0.02f*phi) + dr16_tar.Getvy_left() * cosf(/*M6020_206.theta_planning*/ + 0.02f*phi));
                TransmitSignal();
            }
            
            void stop()
            {
                w = 0.0f;
                vx = 0.0f;
                vy = 0.0f;
                TransmitSignal();
            }

            void keyboard()
            {

            }

        private:
            float vx;
            float vy;
            float w;
    };

    class PIDCalculate
    {
        public:
            void Invoke()
            {
                for(uint8_t i = 0; i < 4; ++i) //µ×ÅÌ½âËãpid
                {
                    Pidout[i] = M3508[i].NormalPID(Motor3508.GetSpeedRpm(i), chassis_pidtarget.GetPidTarget(i));
                }
                //µ×ÅÌ¸úËæpid
                //Pidout[4] = Revolve_PidInstance.NormalPID(, chassis_pidtarget.GetPidTarget(4));
            }

            float GetPidout(uint8_t i) 
            {
                return Pidout[i];
            }
        
        private:
            float Pidout[PIDtargetNum];
    };

    class Send : public PIDCalculate
    {
        public:
            void SendToMotor()
            {
                Invoke();
                for(uint8_t i = 0; i < PIDtargetNum; ++i)
                {
                    MotorSend[i] = GetPidout(i);
                }
                Motor3508.Send0x200(MotorSend[0], MotorSend[1], MotorSend[2], MotorSend[3]);
            }
        
        private:
            float MotorSend[PIDtargetNum];

    };  
}


#endif
