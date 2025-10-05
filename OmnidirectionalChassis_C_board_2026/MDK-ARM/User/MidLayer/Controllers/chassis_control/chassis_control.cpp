include "chassis_control.hpp"

void ChassisControl::modelchoose::ModeSelection()
{
    chass_model.updateState();
    switch(chass_model.getCurrentState())
    {
        case State::model::STOP:

            break;
        case State::model::FOLLOW:
            break;
        case State::model::NOTFOLLOW:
            nofollow(0.005512f);
            break;
        case State::model::KEYBOARD:
            break;
    }
}            
void nofollow(float phi)
{
    VehicleDirection_w  = 8911.0f*dr16_tar.Roller + 2500.0f*dr16_tar.q + 2500.0f*dr16_tar.e;

    VehicleDirection_vx = 8911.0f*(dr16_tar.vx_left *  cosf(M6020_206.theta_planning + 0.02f*phi*VehicleDirection_w) + dr16_tar.vy_left          * sinf(M6020_206.theta_planning + 0.02f*phi*VehicleDirection_w)) 
               + 3000.0f*((dr16_tar.w + dr16_tar.s) *  cosf(M6020_206.theta_planning + 0.02f*phi*VehicleDirection_w) + (dr16_tar.a + dr16_tar.d) * sinf(M6020_206.theta_planning + 0.02f*phi*VehicleDirection_w));
    
    VehicleDirection_vy = 8911.0f*(dr16_tar.vx_left * -sinf(M6020_206.theta_planning + 0.02f*phi*VehicleDirection_w) + dr16_tar.vy_left          * cosf(M6020_206.theta_planning + 0.02f*phi*VehicleDirection_w)) 
               + 3000.0f*((dr16_tar.w + dr16_tar.s) * -sinf(M6020_206.theta_planning + 0.02f*phi*VehicleDirection_w) + (dr16_tar.a + dr16_tar.d) * cosf(M6020_206.theta_planning + 0.02f*phi*VehicleDirection_w));
}

void follow(float phi)
{
    VehicleDirection_w  = 0.0f;
    VehicleDirection_vx = 8911.0f*(dr16.vx_left *  cosf(M6020_206.theta_planning + 0.02f*13.78f) + dr16.vy_left * sinf(M6020_206.theta_planning + 0.02f*13.78f)) + 3000.0f*((dr16.w + dr16.s) *  cosf(M6020_206.theta_planning + 0.02f*13.78f) + (dr16.a + dr16.d) * sinf(M6020_206.theta_planning + 0.02f*13.78f));
    VehicleDirection_vy = 8911.0f*(dr16.vx_left * -sinf(M6020_206.theta_planning + 0.02f*13.78f) + dr16.vy_left * cosf(M6020_206.theta_planning + 0.02f*13.78f)) + 3000.0f*((dr16.w + dr16.s) * -sinf(M6020_206.theta_planning + 0.02f*13.78f) + (dr16.a + dr16.d) * cosf(M6020_206.theta_planning + 0.02f*13.78f));
}

