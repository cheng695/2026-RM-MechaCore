#include "ControlTask.hpp"

Gimbal_FSM gimbal_fsm;
Launch_FSM launch_fsm;
ALG::ADRC::FirstLADRC yaw_ladrc(12.0f, 20.0f, 0.065f, 0.001f, 25000.0f);
ALG::PID::PID yaw_angle_pid(7.0f, 0.0f, 0.0f, 25000.0f, 0.0f, 0.0f);
ALG::PID::PID yaw_velocity_pid(70.0f, 0.0f, 0.0f, 25000.0f, 0.0f, 0.0f);

ALG::PID::PID pitch_angle_pid(0.0f, 0.0f, 0.0f, 12.56f, 0.0f, 0.0f);
ALG::PID::PID pitch_velocity_pid(0.0f, 0.0f, 0.0f, 10.0f, 0.0f, 0.0f);

ALG::PID::PID dial_pid(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f);

ALG::PID::PID surgewheel_pid[2] = {
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f),
    ALG::PID::PID(5.0f, 0.0f, 0.0f, 16385.0f, 2500.0f, 200.0f)
};

ControlTask gimbal_target;
Output_gimbal gimbal_output;
Output_launch launch_output;

void gimbal_fsm_init()
{
    gimbal_fsm.Init();
}

bool check_online()
{
    if(!Motor6020.isConnected(1, 6) || !MotorJ4310.isConnected(1, 2) || !Motor3508.isConnected(1, 1) || !Motor3508.isConnected(1, 4) || !Motor2006.isConnected(1, 3) ||
        !DT7.isConnected() || !HI12.isConnected() || !Aboard.isConnected())
    {
        return false;
    }
    
    return true;
}

void Settarget_gimbal()
{
    gimbal_target.target_yaw += DT7.get_right_x();

    gimbal_target.target_pitch -= DT7.get_right_y();
    if(gimbal_target.target_pitch > 33.0f)
    {
        gimbal_target.target_pitch = 33.0f;
    }
    else if(gimbal_target.target_pitch < -29.0f)
    {
        gimbal_target.target_pitch = -29.0f;
    }
}

void gimbal_stop()
{
    if(MotorJ4310.getIsenable())
    {
        MotorJ4310.Off(BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(false);
    }
    yaw_angle_pid.reset();
    yaw_velocity_pid.reset();
    pitch_angle_pid.reset();
    pitch_velocity_pid.reset();

    gimbal_output.out_yaw = 0.0f;
    gimbal_output.out_pitch = 0.0f;
}

void gimbal_manual()
{
    if(!MotorJ4310.getIsenable())
    {
        MotorJ4310.On(BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(true);
    }
    yaw_ladrc.LADRC_1(132.0f*gimbal_target.target_yaw, HI12.GetGyroRPM(2));

    pitch_angle_pid.UpDate(0.01f*gimbal_target.target_pitch, MotorJ4310.getAddAngleDeg(1));
    pitch_velocity_pid.UpDate(pitch_angle_pid.getOutput(), MotorJ4310.getVelocityRpm(1));

    gimbal_output.out_yaw = yaw_ladrc.GetU();
    gimbal_output.out_pitch = pitch_velocity_pid.getOutput();
}

void gimbal_vision()
{
    if(!MotorJ4310.getIsenable())
    {
        MotorJ4310.On(BSP::Motor::DM::MIT);
        MotorJ4310.setIsenable(true);
    }
    yaw_angle_pid.UpDate(0.01f*gimbal_target.target_yaw, HI12.GetAddYaw());
    yaw_velocity_pid.UpDate(yaw_angle_pid.getOutput(), HI12.GetGyroRPM(2));

    pitch_angle_pid.UpDate(0.01f*gimbal_target.target_pitch, MotorJ4310.getAddAngleDeg(1));
    pitch_velocity_pid.UpDate(pitch_angle_pid.getOutput(), MotorJ4310.getVelocityRpm(1));

    gimbal_output.out_yaw = yaw_velocity_pid.getOutput();
    gimbal_output.out_pitch = pitch_velocity_pid.getOutput();
}

void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online) 
{   
    gimbal_fsm.StateUpdate(left_sw, right_sw, is_online);
    Settarget_gimbal();

    switch(gimbal_fsm.Get_Now_State()) 
    {
        case STOP:
            gimbal_stop();
            break;
        case MANUAL:
            gimbal_manual();
            break;
        case VISION:
            gimbal_vision();
            break;
        case KEYBOARD:
            //chassis_keyboard_control();
            break;
        default:
            gimbal_stop();
            break;
    }
}






void Settarget_launch()
{
    gimbal_target.target_dial += DT7.get_scroll_();

    gimbal_target.target_surgewheel[0] = 5500.0f;
    gimbal_target.target_surgewheel[1] = -5500.0f;
}


void launch_stop()
{
    dial_pid.reset();
    for(int i = 0; i < 2; i++)
    {
        surgewheel_pid[i].reset();
    }

    launch_output.out_surgewheel[0] = 0.0f;
    launch_output.out_surgewheel[1] = 0.0f;
    launch_output.out_dial = 0.0f;
}

void launch_ceasefire()
{
    dial_pid.UpDate(0.0f, Motor2006.getVelocityRpm(1));
    surgewheel_pid[0].UpDate(gimbal_target.target_surgewheel[0], Motor2006.getVelocityRpm(1));
    surgewheel_pid[1].UpDate(gimbal_target.target_surgewheel[1], Motor2006.getVelocityRpm(2));

    launch_output.out_dial = dial_pid.getOutput();
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
}

void launch_rapidfire()
{
    dial_pid.UpDate(4400.0f*gimbal_target.target_dial, Motor2006.getVelocityRpm(1));
    surgewheel_pid[0].UpDate(gimbal_target.target_surgewheel[0], Motor2006.getVelocityRpm(1));
    surgewheel_pid[1].UpDate(gimbal_target.target_surgewheel[1], Motor2006.getVelocityRpm(2));

    launch_output.out_dial = dial_pid.getOutput();
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
}

void launch_singalshot()
{
    dial_pid.UpDate(360.0f*gimbal_target.target_dial, Motor2006.getAddAngleDeg(1));
    surgewheel_pid[0].UpDate(gimbal_target.target_surgewheel[0], Motor2006.getVelocityRpm(1));
    surgewheel_pid[1].UpDate(gimbal_target.target_surgewheel[1], Motor2006.getVelocityRpm(2));

    launch_output.out_dial = dial_pid.getOutput();
    launch_output.out_surgewheel[0] = surgewheel_pid[0].getOutput();
    launch_output.out_surgewheel[1] = surgewheel_pid[1].getOutput();
}

void main_loop_launch(uint8_t left_sw, uint8_t right_sw, bool is_online)
{
    launch_fsm.StateUpdate(left_sw, right_sw, is_online);
    Settarget_launch();

    switch(launch_fsm.Get_Now_State()) 
    {
        case LAUNCH_STOP:
            launch_stop();
            break;
        case LAUNCH_CEASEFIRE:
            launch_ceasefire();
            break;
        case LAUNCH_RAPIDFIRE:
            launch_rapidfire();
            break;
        case LAUNCH_SINGALSHOT:
            launch_singalshot();
            break;
        case LAUNCH_KEYBOARD:
            //chassis_keyboard_control();
            break;
        default:
            launch_stop();
    }
}


extern "C"{
void control(void const * argument)
{
    // 初始化蜂鸣器管理器
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    MotorJ4310.Off(BSP::Motor::DM::MIT);
    gimbal_target.target_pitch = MotorJ4310.getAddAngleDeg(1);
    gimbal_fsm_init();
    for(;;)
    {
        // 更新蜂鸣器管理器，处理队列中的响铃请求
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online());

        osDelay(1);
    } 
}
}