#include "chassis_control.hpp"
#include "../User/MidLayer/Managers/state_manager/state.hpp"
#include "../User/MidLayer/Controllers/chassis_control/PidTarget.hpp"

extern State::model chass_model;

PidTarget::pidtarget chassis_pidtarget;
ChassisControl::modelchoose model_choose;
ChassisControl::Send motor_send;
ChassisControl::PIDCalculate pid_calculate;

void ChassisControl::modelchoose::ModeSelection()
{
    chass_model.updateState();
    if(chass_model.isStateChanged())
    {
        switch(chass_model.getLastState())
        {
            case State::model::STOP:
                
                break;
            case State::model::FOLLOW:

                break;
            case State::model::NOTFOLLOW:

                break;
            case State::model::SHOT_FOLLOW:

                break;
            case State::model::SHOT_Rotate:

                break;
            case State::model::VISION_NOTSHOT:

                break;
            case State::model::VISION_SHOT:

                break;
            case State::model::KEYBOARD:

                break;
        }
    }
    else
    {
        switch(chass_model.getCurrentState())
        {
            case State::model::STOP:
                model_choose.stop();
                break;
            case State::model::FOLLOW:
                model_choose.follow(13.78f);
                break;
            case State::model::NOTFOLLOW:
                model_choose.nofollow(0.005512f);
                break;
            case State::model::SHOT_FOLLOW:
                model_choose.follow(13.78f);
                break;
            case State::model::SHOT_Rotate:
                model_choose.rotate_follow(13.78f);
                break;
            case State::model::VISION_NOTSHOT:
                model_choose.nofollow(0.005512f);
                break;
            case State::model::VISION_SHOT:
                model_choose.nofollow(0.005512f);
                break;
            case State::model::KEYBOARD:
                model_choose.keyboard();
                break;
        }
    }
}
