#include "ControlTask.hpp"

/**
 * @brief 鍒濆鍖?
 */
extern bool alphabet[28];

/* 瀹忓畾涔?--------------------------------------------------------------------------------------------------*/
#define PI_ 3.1415926535897932384626433832795
#define STANDARD -2.39347f  // 姝ｆ柟鍚戯紙寮у害锛?

/* 鏈夐檺鐘舵€佹満 -----------------------------------------------------------------------------------------------*/
Chassis_FSM chassis_fsm;    // 搴曠洏鏈夐檺鐘舵€佹満

namespace
{
    constexpr int16_t kDialJamReverseTorque = -20;
    constexpr uint32_t kDialJamReverseMs = 60;
    constexpr uint32_t kDialJamClearErrWaitMs = 10;
    constexpr uint32_t kDialJamOffWaitMs = 10;
    constexpr uint32_t kDialJamOnWaitMs = 20;
    constexpr uint32_t kDialJamRecoverReverseMs = 60;
    constexpr uint32_t kDialJamExitHoldMs = 80;

    enum class DialJamRecoverState : uint8_t
    {
        Idle = 0,
        Reverse,
        ClearErrWait,
        OffWait,
        OnWait,
        RecoverReverse,
        Hold,
    };

    DialJamRecoverState g_dial_jam_recover_state = DialJamRecoverState::Idle;
    uint32_t g_dial_jam_recover_tick = 0;

    void setDialJamRecoverState(DialJamRecoverState state)
    {
        g_dial_jam_recover_state = state;
        g_dial_jam_recover_tick = HAL_GetTick();
    }

    void resetDialJamRecoverState()
    {
        g_dial_jam_recover_state = DialJamRecoverState::Idle;
        g_dial_jam_recover_tick = 0;
    }
}

/* 搴曠洏瑙ｇ畻 -------------------------------------------------------------------------------------------------*/
float wheel_azimuth[4] = {-PI_/4, -3*PI_/4, 3*PI_/4, PI_/4};      // 杞畨瑁呮柟浣嶈 (姣忎釜浣嶇疆杞粍鐨勮嚜韬粴鍚?
float wheel_direction[4] = {-3*PI_/4, 3*PI_/4, PI_/4, -PI_/4};    // 杞畨瑁呬綅缃 (0:FL, 1:BL, 2:BR, 3:FR)
Alg::CalculationBase::Omni_IK omni_ik(0.24f, 0.07f, wheel_azimuth, wheel_direction);        // 杩愬姩瀛﹂€嗚В绠?
Alg::CalculationBase::Omni_FK omni_fk(0.24f, 0.07f, 4.0f, wheel_azimuth, wheel_direction);  // 杩愬姩瀛︽瑙ｇ畻

/* 鎺у埗鍣?--------------------------------------------------------------------------------------------------*/
// 鐢ㄤ簬杩愬姩瀛﹂€嗚В锛岃疆鍚戜綔涓鸿ˉ鍋胯緭鍑?
ALG::PID::PID wheel_pid[4] = {
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 杞悜閫熷害pid 1鍙疯疆
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 杞悜閫熷害pid 2鍙疯疆
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 100.0f),     // 杞悜閫熷害pid 3鍙疯疆
    ALG::PID::PID(700.0f, 0.5f, 0.0f, 16384.0f, 2500.0f, 150.0f)      // 杞悜閫熷害pid 4鍙疯疆
};  

ALG::PID::PID follow_pid(8.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);  // 閫熷害鐜痯id 鐢ㄤ簬搴曠洏璺熼殢

ALG::PID::PID energy_pid[2] = {
    ALG::PID::PID(15.0f, 0.0f, 10.0f, 16384.0f, 0.0f, 0.0f),     // 瀵岃冻鐜?
    ALG::PID::PID(4.0f, 0.0f, 10.0f, 16384.0f, 0.0f, 0.0f)      // 璐洶鐜?
};

ALG::PID::PID dial_angle_pid(45.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);
ALG::PID::PID dial_velocity_pid(5.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);
ALG::PID::PID dial_pid(0.0f, 0.0f, 0.0f, 16384.0f, 2500.0f, 100.0f);

/* 鍔熺巼鎺у埗 -------------------------------------------------------------------------------------------------*/
ALG::PowerControl::PowerControl<4> power3508;   // 3508鍔熺巼鎺у埗绠楁硶
ALG::PowerControl::EnergyRing energy_ring(1288.5f, 250.0f, 48.0f);  // 鑳介噺鐜?
ALG::PowerControl::PowerControlStrategy power_strategy(1288.5f); // 涓婇檺鍔熺巼鍜屽墿浣欒兘閲忛€昏緫澶勭悊
float coefficients3508[6] = { 2.144951, -0.002828, 0.000025, 0.016525,  0.115369, 0.000015 };   // 3508

/* 鏈熸湜鍊间笌杈撳嚭 ----------------------------------------------------------------------------------------------*/
Alg::Utility::SlopePlanning omni_target[3] = {
    Alg::Utility::SlopePlanning(0.015f, 0.015f),    // X杞存枩鍧¤鍒?
    Alg::Utility::SlopePlanning(0.015f, 0.015f),    // Y杞存枩鍧¤鍒?
    Alg::Utility::SlopePlanning(0.05f, 0.05f)       // Z杞存棆杞枩鍧¤鍒?
};

ControlTask chassis_target;     // 搴曠洏鐩爣
Output_chassis chassis_output;  // 搴曠洏杈撳嚭


/**
 * @brief 璧吂杞搴?
 * 
 * @param fire_hz 
 * @return float 
 */
float hz_to_angle(float fire_hz)
{
    const int slots_per_rotation = 10;
    const float angle_per_slot = 360.0f / slots_per_rotation;
    const float control_period = 0.001f;

    float angle_per_frame = (fire_hz * angle_per_slot) * control_period;
    return angle_per_frame;
}

/**
 * @brief 搴曠洏鎺у埗閫昏緫
 */
/* 搴曠洏閮ㄥ垎 ---------------------------------------------------------------------------------------------------*/

/**
 * @brief 妫€鏌ユ墍鏈夊叧閿澶囨槸鍚﹀湪绾?
 * 
 * @return true 璁惧鍏ㄩ儴鍦ㄧ嚎
 * @return false 瀛樺湪绂荤嚎璁惧
 */
bool check_online()
{
    bool isconnected = true;
    for(int i = 0; i < 4; i++)
    {
        if(!Motor3508.isConnected(i+1, i+1))
        {
            isconnected = false;
        }
    }

    if(/*!Cboard.isConnected() ||*/ !DT7.isConnected())
    {
        isconnected = false;
    }

    if(RM_RefereeSystem::RM_RefereeSystemDir())
    {
        // isconnected = false;
    }
    
    if(!isconnected)
    {
        return false;
    }

 
    
    return true;
}

/**
 * @brief 鐘舵€佹満鍒濆鍖?
 */
void fsm_init()
{
    chassis_fsm.Init();
}

/**
 * @param theta 浜戝彴瑙掑害锛堝崟浣?寮у害锛?
 * @param vx 绗涘崱灏斿潗鏍囩郴鐨刋鏂瑰悜杈撳叆锛堝乏鍙筹級
 * @param vy 绗涘崱灏斿潗鏍囩郴鐨刌鏂瑰悜杈撳叆锛堝墠鍚庯級
 * @param phi 瑙掑害鍋忓樊琛ュ伩锛堝崟浣?寮у害锛?
 * @param out_x [杈撳嚭] 璁＄畻鏃嬭浆鐭╅樀鍚庣殑X鏂瑰悜閫熷害锛堝墠鍚庯級
 * @param out_y [杈撳嚭] 璁＄畻鏃嬭浆鐭╅樀鍚庣殑Y鏂瑰悜閫熷害锛堝乏鍙筹級
 */
void CalculateTranslation_xy(float theta, float vx, float vy, float phi, float *out_vx, float *out_vy, float psi)
{
    // 灏嗕簯鍙板弽棣堣鍑忓幓姝ｅ墠鏂归浂浣嶏紝鍐嶅彔鍔犻澶栬ˉ鍋胯
    theta = theta - STANDARD + phi;

    // 杩囬浂澶勭悊 (褰掍竴鍖栧埌 -PI ~ PI)
    theta = fmod(theta, 2 * PI_);
    if (theta > PI_) theta -= 2 * PI_;
    else if (theta < -PI_) theta += 2 * PI_;

    float s = sinf(theta + psi);
    float c = cosf(theta + psi);
    // 鎺у埗閲忚緭鍏?(浜戝彴绯?
    float raw_vx = 4.5f * vy;  // raw_vx鏄満鍣ㄤ汉鍧愭爣绯诲墠鍚庢柟鍚?
    float raw_vy = -4.5f * vx; // 妯Щ鏂瑰悜鍙栧弽锛屼慨姝ｅ乏鍙充笌閬ユ帶杈撳叆鐩稿弽鐨勯棶棰?
    // 鏃嬭浆鍒板簳鐩樼郴
    *out_vx = raw_vx * c - raw_vy * s; 
    *out_vy = raw_vx * s + raw_vy * c;
}


/**
 * @brief 搴曠洏璺熼殢w瑙勫垝
 */
void CalculateFollow()
{
    float follow_error = STANDARD - Cboard.GetYawAngle();
    // 褰掍竴鍖栧鐞?
    while (follow_error > 1.5707963267f) follow_error -= 2 * 1.5707963267f;
    while (follow_error < -1.5707963267f) follow_error += 2 * 1.5707963267f;

    // 姝诲尯澶勭悊 (闃叉灏忚宸渿鑽?
    if(fabs(follow_error) < 0.01f) follow_error = 0.0f;
    
    follow_pid.UpDate(0.0f, follow_error);
}

/**
 * @brief 璁剧疆搴曠洏鐨勭洰鏍囧€?
 * 
 * 鏍规嵁褰撳墠宸ヤ綔妯″紡璁剧疆鐩稿簲鐨勭洰鏍囧€?
 */
void SetTarget()
{
    // 璁剧疆姝诲尯
    DT7.SetDeadzone(20.0f);
    // 搴曠洏
    switch(chassis_fsm.Get_Now_State()) 
    {
        case STOP:  // 鍋滄妯″紡
            chassis_target.target_translation_x = 0.0f;
            chassis_target.target_translation_y = 0.0f;
            chassis_target.target_rotation = 0.0f;
            break;
        case FOLLOW:  // 搴曠洏璺熼殢
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)  // 閿紶妯″紡鐨勫簳鐩樿窡闅?
            {
                float vx, vy;
                float psi = 0.0f; // 涓嶅皬闄€铻虹殑鏃跺€欎笉琛ュ伩
                
                float vx_Handle, vy_Handle, vw_Handle;
                vx_Handle = alphabet[22] - alphabet[18];
                vy_Handle = alphabet[3] - alphabet[0];
                vw_Handle = alphabet[23];
                
                // 灏忛檧铻烘椂杩涜鐩镐綅琛ュ伩
                if (vw_Handle != 0.0f) 
                {
                    //psi = -0.08f*13.00f*vw_Handle;
                    psi = 0.0f;
                }
                
                CalculateTranslation_xy(Cboard.GetYawAngle(), vy_Handle, vx_Handle, 0.0f, &vx, &vy, psi);  // 璁＄畻鏃嬭浆鐭╅樀
                omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx()); // 鏂滃潯瑙勫垝 X鏂瑰悜锛堝墠鍚庯級
                omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy()); // 鏂滃潯瑙勫垝 Y鏂瑰悜锛堝乏鍙筹級
                chassis_target.target_translation_x = omni_target[0].GetOut();    // X鏂瑰悜鏈熸湜锛堝墠鍚庯級
                chassis_target.target_translation_y = omni_target[1].GetOut();    // Y鏂瑰悜鏈熸湜锛堝乏鍙筹級
                if(alphabet[23])   // 灏忛檧铻?
                {
                    omni_target[2].TIM_Calculate_PeriodElapsedCallback(13.00f * vw_Handle, omni_fk.GetChassisVw()); 
                    chassis_target.target_rotation = omni_target[2].GetOut(); // 灏忛檧铻虹殑鏈熸湜
                }
                else    // 搴曠洏璺熼殢
                {
                    CalculateFollow();  // 璁＄畻搴曠洏璺熼殢鐨勮鍒?
                    chassis_target.target_rotation = follow_pid.getOutput();    // Z杞村簳鐩樿窡闅忔棆杞湡鏈?
                }                
            }   
            else    // 閬ユ帶鍣ㄦā寮忕殑搴曠洏璺熼殢
            {
                float vx, vy;
                float psi = 0.0f;   // 涓嶅皬闄€铻虹殑鏃跺€欎笉琛ュ伩
                // 鍙湪闄€铻烘ā寮忎笅杩涜鐩镐綅琛ュ伩
                if (Cboard.GetScroll() == false && fabs(DT7.get_scroll_()) > 0.05f) 
                {
                    // psi = -0.08f * 13.00f * DT7.get_scroll_();
                    psi = 0.0f;
                }

                CalculateTranslation_xy(Cboard.GetYawAngle(), DT7.get_left_x(), DT7.get_left_y(), 0.0f, &vx, &vy, psi);  // 璁＄畻鏃嬭浆鐭╅樀
                omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx()); // 鏂滃潯瑙勫垝 X鏂瑰悜锛堝墠鍚庯級
                omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy()); // 鏂滃潯瑙勫垝 Y鏂瑰悜锛堝乏鍙筹級
                chassis_target.target_translation_x = omni_target[0].GetOut();    // X鏂瑰悜鏈熸湜锛堝墠鍚庯級
                chassis_target.target_translation_y = omni_target[1].GetOut();    // Y鏂瑰悜鏈熸湜锛堝乏鍙筹級
                if(Cboard.GetScroll() == true)  // 涓嶈兘灏忛檧铻?
                {
                    CalculateFollow();  // 璁＄畻搴曠洏璺熼殢鐨勮鍒?
                    chassis_target.target_rotation = follow_pid.getOutput();    // Z杞村簳鐩樿窡闅忔棆杞湡鏈?
                }
                else    // 鍙互灏忛檧铻?
                {
                    if(fabs( DT7.get_scroll_() ) > 0.05f)   // 灏忛檧铻?
                    {
                        omni_target[2].TIM_Calculate_PeriodElapsedCallback(13.00f * DT7.get_scroll_(), omni_fk.GetChassisVw()); 
                        chassis_target.target_rotation = omni_target[2].GetOut(); // 灏忛檧铻虹殑鏈熸湜
                    }
                    else    // 搴曠洏璺熼殢
                    {
                        CalculateFollow();  // 璁＄畻搴曠洏璺熼殢鐨勮鍒?
                        chassis_target.target_rotation = follow_pid.getOutput();    // Z杞村簳鐩樿窡闅忔棆杞湡鏈?
                    }
                }
            }
            break;
        case NOTFOLLOW:
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)  // 閿紶妯″紡
            {
                float vx, vy;
                float psi = 0.0f;   // 涓嶅皬闄€铻虹殑鏃跺€欎笉琛ュ伩

                float vx_Handle, vy_Handle, vw_Handle;
                vx_Handle = alphabet[22] - alphabet[18];
                vy_Handle = alphabet[3] - alphabet[0];
                vw_Handle = alphabet[23];
                
                // 灏忛檧铻烘椂杩涜鐩镐綅琛ュ伩
                if (vw_Handle != 0.0f) 
                {
                    //psi = -0.08f*13.00f*vw_Handle;
                    psi = 0.0f;
                }
                
                CalculateTranslation_xy(Cboard.GetYawAngle(), vy_Handle, vx_Handle, 0.0f, &vx, &vy, psi);  // 璁＄畻鏃嬭浆鐭╅樀
                omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx()); // 鏂滃潯瑙勫垝 X鏂瑰悜锛堝墠鍚庯級
                omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy()); // 鏂滃潯瑙勫垝 Y鏂瑰悜锛堝乏鍙筹級
                chassis_target.target_translation_x = omni_target[0].GetOut();    // X鏂瑰悜鏈熸湜锛堝墠鍚庯級
                chassis_target.target_translation_y = omni_target[1].GetOut();    // Y鏂瑰悜鏈熸湜锛堝乏鍙筹級
                omni_target[2].TIM_Calculate_PeriodElapsedCallback(13.00f * vw_Handle, omni_fk.GetChassisVw());
                chassis_target.target_rotation = omni_target[2].GetOut(); // 灏忛檧铻虹殑鏈熸湜
            }   
            else    // 閬ユ帶鍣ㄦā寮?
            {
                float vx, vy;
                float psi = 0.0f;   // 涓嶅皬闄€铻虹殑鏃跺€欎笉琛ュ伩
                // 鍙湪闄€铻烘ā寮忎笅杩涜鐩镐綅琛ュ伩
                if (Cboard.GetScroll() == false && fabs(DT7.get_scroll_()) > 0.05f)
                { 
                    // psi = -0.08f * 13.00f * DT7.get_scroll_();
                    psi = 0.0f;
                }
                CalculateTranslation_xy(Cboard.GetYawAngle(), DT7.get_left_x(), DT7.get_left_y(), 0.0f, &vx, &vy, psi);  // 璁＄畻鏃嬭浆鐭╅樀
                omni_target[0].TIM_Calculate_PeriodElapsedCallback(vx, omni_fk.GetChassisVx()); // 鏂滃潯瑙勫垝 X鏂瑰悜锛堝墠鍚庯級
                omni_target[1].TIM_Calculate_PeriodElapsedCallback(vy, omni_fk.GetChassisVy()); // 鏂滃潯瑙勫垝 Y鏂瑰悜锛堝乏鍙筹級
                chassis_target.target_translation_x = omni_target[0].GetOut();    // X鏂瑰悜鏈熸湜锛堝墠鍚庯級
                chassis_target.target_translation_y = omni_target[1].GetOut();    // Y鏂瑰悜鏈熸湜锛堝乏鍙筹級
                if(Cboard.GetScroll() == true)  // 涓嶈兘灏忛檧铻?
                {
                    chassis_target.target_rotation = 0.0f;  // Z杞翠笉鏃嬭浆
                }
                else    // 鍙互灏忛檧铻?
                {
                    omni_target[2].TIM_Calculate_PeriodElapsedCallback(13.00f * DT7.get_scroll_(), omni_fk.GetChassisVw());
                    chassis_target.target_rotation = omni_target[2].GetOut(); // 灏忛檧铻虹殑鏈熸湜
                }
            }
            break;
        default:
            chassis_target.target_translation_x = 0.0f;
            chassis_target.target_translation_y = 0.0f;
            chassis_target.target_rotation = 0.0f;
            break;
    }
}

void SetTarget_Launch()
{
    static int last_launch_state = -1;
    DT7.SetDeadzone(20.0f);
    switch(Cboard.GetLaunchState())
    {
         case 0: // STOP
            chassis_target.target_dial = MotorLK4005.getAddAngleDeg(1);
            break;
        case 1: // 鍋滅伀
            chassis_target.target_dial = MotorLK4005.getAddAngleDeg(1);
            break;
        case 2: // 鍗曞彂
            // 杩涚姸鎬佹椂鍚屾涓€娆＄洰鏍囧€?
            if (last_launch_state != 2) 
            {
                chassis_target.target_dial = MotorLK4005.getAddAngleDeg(1);
            }
            
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)
            {
                static bool last_mouse_left = false;
                if (alphabet[26] && !last_mouse_left)   // 涓婂崌娌挎娴?
                {
                    chassis_target.target_dial += 36.0f; // 鍙噺涓€娆?
                }
                last_mouse_left = alphabet[26];
            }
            else
            {
                static bool last_scroll_active = false;
                
                // 闃堝€煎垽鏂?
                float scroll_val = DT7.get_scroll_();
                bool current_scroll_active = (fabs(scroll_val) > 0.8f);

                // 涓嬮檷娌挎娴嬶細浠?楂樹綅 鍙?浣庝綅
                if (last_scroll_active && !current_scroll_active)
                {
                    chassis_target.target_dial += 36.0f; // 鍙噺涓€娆?
                }
                last_scroll_active = current_scroll_active;
            }
            break;
        case 3: // 杩炲彂
            if (last_launch_state != 3) 
            {
                chassis_target.target_dial = MotorLK4005.getAddAngleDeg(1);
            }
            if(DT7.get_s1() == 3 && DT7.get_s2() == 3)
            {
                // chassis_target.target_dial -= DT7.get_mouseLeft() * hz_to_angle(heat_control.GetNowFire());
                chassis_target.target_dial += DT7.get_mouseLeft() * hz_to_angle(13.0f);
            }
            else
            {
                // chassis_target.target_dial -= DT7.get_scroll_() * hz_to_angle(heat_control.GetNowFire());
                chassis_target.target_dial += DT7.get_scroll_() * hz_to_angle(13.0f);
            }
            break;
        case 4: // 鍗″脊
            chassis_target.target_dial = MotorLK4005.getAddAngleDeg(1);
            break;
        default:
            chassis_target.target_dial = MotorLK4005.getAddAngleDeg(1);
            break;
    }

    last_launch_state = Cboard.GetLaunchState();
}
/**
 * @brief 搴曠洏鍋滄妯″紡鎺у埗鍑芥暟
 * 
 * 閲嶇疆鎺у埗鍣ㄥ弬鏁?
 */
void chassis_stop()
{
    for(int i = 0; i < 4; i++)
    {
        // 閲嶇疆鎺у埗鍣?
        wheel_pid[i].reset();
        // 杈撳嚭缃浂
        chassis_output.out_wheel[i] = 0.0f;
    }
}

/**
 * @brief 搴曠洏涓嶈窡闅忔帶鍒跺嚱鏁?
 * 
 * 瀹炵幇搴曠洏涓嶈窡闅忔帶鍒?
 */
void chassis_notfollow()
{
    // 姝ｈ繍鍔ㄥ瑙ｇ畻褰撳墠搴曠洏鏁翠綋閫熷害锛涢€嗚繍鍔ㄥ瑙ｇ畻鐢垫満杞€?
    omni_fk.OmniForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    omni_ik.OmniInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 1.0f, 1.0f);
    
    for(int i = 0; i < 4; i++)
    {
        // 閫嗚繍鍔ㄥ鐩稿叧 閫氳繃PID绠楄疆鍚戣ˉ鍋?
        wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRads(i+1));
        chassis_output.out_wheel[i] = wheel_pid[i].getOutput();
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
    
    // 鍔熺巼鎺у埗
    bool isSupercapOnline = supercap.isConnected(); // 鐢靛杩炴帴鐘舵€?
    bool isRefereeOnline = !RM_RefereeSystem::RM_RefereeSystemDir(); // 瑁佸垽绯荤粺杩炴帴鐘舵€?
    
    supercap.setSupercapOnline(isSupercapOnline);
    supercap.setRefereeOnline(isRefereeOnline);

    // 鏇存柊鍔熺巼绛栫暐
    power_strategy.Update(isSupercapOnline, isRefereeOnline, 
                          (float)ext_power_heat_data_0x0201.chassis_power_limit, 
                          ext_power_heat_data_0x0202.chassis_power_buffer, 
                          supercap.GetCurrentEnergy(), alphabet);

    float input_limit = power_strategy.GetInputLimit(); // 鍩虹涓婇檺鍔熺巼
    float input_energy = power_strategy.GetInputEnergy();   // 鍓╀綑鑳介噺

    energy_pid[0].UpDate(sqrtf(energy_ring.GetAbundanceLine()), sqrtf(input_energy));   // 瀵岃冻鐜?
    energy_pid[1].UpDate(sqrtf(energy_ring.GetPovertyLine()), sqrtf(input_energy));     // 璐洶鐜?
    // 鑳介噺鐜€昏緫
    energy_ring.energyring(energy_pid[0].getOutput(), energy_pid[1].getOutput(), input_limit, input_energy, alphabet[26], alphabet[27]);
    float PowerMax = energy_ring.GetPowerMax(); // 鏈€缁堜笂闄愬姛鐜?

    supercap.setInstruction(0); // 寮€鍚秴鐢?
    supercap.setRatedPower(input_limit);    // 缁欒秴鐢佃缃鍒ょ郴缁熶笂闄愬姛鐜?
    supercap.setBufferEnergy(ext_power_heat_data_0x0202.chassis_power_buffer);  // 缁欒秴鐢佃缃墿浣欑紦鍐茶兘閲?

    float I3508[4], I_other[4], V3508[4];   // 蹇呰鍙傛暟
    for(int i = 0; i < 4; i++)
    {
        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;    // 3508鐢垫満鐢垫祦锛堣В绠楁杈撳嚭鐢垫祦锛?
        I_other[i] = 0.0f;                                          // 3508鐢垫満鐢垫祦锛堥潪瑙ｇ畻娆茶緭鍑虹數娴侊級
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f);  // 3508鐢垫満閫熷害锛堝綋鍓嶉€熷害锛?
    }
    float pmax3508 = PowerMax; // 3508鐢垫満鍚冩弧
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508); // 3508鐢垫満鍔熺巼鎺у埗锛堣“鍑忕數娴佹硶锛?

    // 搴曠洏杈撳嚭
    for(int i = 0; i < 4; i++)
    {
        //chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;
    }
}

/**
 * @brief 搴曠洏璺熼殢鎺у埗鍑芥暟
 * 
 * 瀹炵幇搴曠洏璺熼殢鎺у埗
 */
void chassis_follow()
{   
    // 姝ｈ繍鍔ㄥ瑙ｇ畻褰撳墠搴曠洏鏁翠綋閫熷害锛涢€嗚繍鍔ㄥ瑙ｇ畻鐢垫満杞€?
    omni_fk.OmniForKinematics(Motor3508.getVelocityRads(1), Motor3508.getVelocityRads(2), Motor3508.getVelocityRads(3), Motor3508.getVelocityRads(4));
    omni_ik.OmniInvKinematics(chassis_target.target_translation_x, chassis_target.target_translation_y, chassis_target.target_rotation, 0.0f, 1.0f, 1.0f);
    
    for(int i = 0; i < 4; i++)
    {
        // 閫嗚繍鍔ㄥ鐩稿叧 閫氳繃PID绠楄埖鍚戣緭鍑哄拰杞悜琛ュ伩
        wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRads(i+1));
        chassis_output.out_wheel[i] = wheel_pid[i].getOutput();
        chassis_output.out_wheel[i] = std::clamp(chassis_output.out_wheel[i], -16384.0f, 16384.0f);
    }
    
    // 鍔熺巼鎺у埗
    bool isSupercapOnline = supercap.isConnected();
    bool isRefereeOnline = !RM_RefereeSystem::RM_RefereeSystemDir();

    supercap.setSupercapOnline(isSupercapOnline);
    supercap.setRefereeOnline(isRefereeOnline);

    // 鏇存柊鍔熺巼绛栫暐
    power_strategy.Update(isSupercapOnline, isRefereeOnline, 
                          (float)ext_power_heat_data_0x0201.chassis_power_limit, 
                          ext_power_heat_data_0x0202.chassis_power_buffer, 
                          supercap.GetCurrentEnergy(), alphabet);

    float input_limit = power_strategy.GetInputLimit(); // 鍩虹涓婇檺鍔熺巼
    float input_energy = power_strategy.GetInputEnergy();   // 鍓╀綑鑳介噺

    energy_pid[0].UpDate(sqrtf(energy_ring.GetAbundanceLine()), sqrtf(input_energy));   // 瀵岃冻鐜?
    energy_pid[1].UpDate(sqrtf(energy_ring.GetPovertyLine()), sqrtf(input_energy));     // 璐洶鐜?
    // 鑳介噺鐜€昏緫
    energy_ring.energyring(energy_pid[0].getOutput(), energy_pid[1].getOutput(), input_limit, input_energy, alphabet[26], alphabet[27]);
    float PowerMax = energy_ring.GetPowerMax(); // 鏈€缁堜笂闄愬姛鐜?
    
    supercap.setInstruction(0); // 寮€鍚秴鐢?
    supercap.setRatedPower(input_limit);    // 缁欒秴鐢佃缃鍒ょ郴缁熶笂闄愬姛鐜?
    supercap.setBufferEnergy(ext_power_heat_data_0x0202.chassis_power_buffer);  // 缁欒秴鐢佃缃墿浣欑紦鍐茶兘閲?

    float I3508[4], I_other[4], V3508[4];   // 蹇呰鍙傛暟
    for(int i = 0; i < 4; i++)
    {
        I3508[i] = chassis_output.out_wheel[i] * 20.0f/16384.0f;    // 3508鐢垫満鐢垫祦锛堣В绠楁杈撳嚭鐢垫祦锛?
        I_other[i] = 0.0f;                                          // 3508鐢垫満鐢垫祦锛堥潪瑙ｇ畻娆茶緭鍑虹數娴侊級
        V3508[i] = Motor3508.getVelocityRads(i+1)*(268.0f / 17.0f); // 3508鐢垫満閫熷害锛堝綋鍓嶉€熷害锛?
    }
    float pmax3508 = PowerMax; // 3508鐢垫満鍚冩弧
    power3508.DecayingCurrent(I3508, V3508, coefficients3508, I_other, 0.0f/*(-2.144951*3.0f)*/, pmax3508); // 3508鐢垫満鍔熺巼鎺у埗锛堣“鍑忕數娴佹硶锛?

    // 搴曠洏杈撳嚭
    for(int i = 0; i < 4; i++)
    {
        //chassis_output.out_wheel[i] = power3508.getCurrentCalculate(i) * 16384.0f/20.0f;
    }
}

/**
 * @brief 搴曠洏涓绘帶鍒跺惊鐜?
 * 
 * @param left_sw 閬ユ帶鍣ㄥ乏寮€鍏崇姸鎬?
 * @param right_sw 閬ユ帶鍣ㄥ彸寮€鍏崇姸鎬?
 * @param is_online 璁惧鍦ㄧ嚎鐘舵€?
 */
void main_loop_gimbal(uint8_t left_sw, uint8_t right_sw, bool is_online, bool *alphabet) 
{   
    chassis_fsm.StateUpdate(left_sw, right_sw, is_online, alphabet);
    SetTarget();

    switch(chassis_fsm.Get_Now_State()) 
    {
        case STOP:
            chassis_stop();
            break;
        case FOLLOW:
            chassis_follow();
            break;
        case NOTFOLLOW:
            chassis_notfollow();
            break;
        default:
            chassis_stop();
            break;
    }
}


/* 鍙戝皠鏈烘瀯閮ㄥ垎 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 鍙戝皠鏈烘瀯鍋滄妯″紡鎺у埗鍑芥暟
 * 
 * 閲嶇疆鎵€鏈夋帶鍒跺櫒骞惰缃緭鍑轰负0
 */
void launch_velocity()
{
    // 閫熷害鐜湡鏈涘€肩疆闆讹紝蹇€熷仠锟?
    dial_pid.UpDate(chassis_target.target_dial, MotorLK4005.getVelocityRpm(1));
    chassis_output.out_dial = 0.0f;
}

/**
 * @brief 鍙戝皠鏈烘瀯鍗曞彂浠ュ強杩炲彂妯″紡鎺у埗鍑芥暟
 * 
 * 鎷ㄧ洏鎸夎搴︽帶鍒讹紝鎽╂摝杞繚鎸佽瀹氳浆锟?
 */
void launch_angle()
{
    // 鎷ㄧ洏瑙掑害锟?
    dial_angle_pid.UpDate(chassis_target.target_dial, MotorLK4005.getAddAngleDeg(1));
    dial_velocity_pid.UpDate(dial_angle_pid.getOutput(), MotorLK4005.getVelocityRpm(1));
    chassis_output.out_dial = dial_velocity_pid.getOutput() * 2000.0f/16384.0f;
}

/**
 * @brief 鍙戝皠鏈烘瀯鍗″脊鎺у埗鍑芥暟
 * 
 * 鍗″脊鐨勬椂鍊欒緭鍑哄弽鍚戝姏锟?
 */
void launch_jam()
{
    uint32_t now = HAL_GetTick();

    if (g_dial_jam_recover_state == DialJamRecoverState::Idle)
    {
        setDialJamRecoverState(DialJamRecoverState::Reverse);
    }

    switch (g_dial_jam_recover_state)
    {
        case DialJamRecoverState::Reverse:
            chassis_output.out_dial = kDialJamReverseTorque;
            if (now - g_dial_jam_recover_tick >= kDialJamReverseMs)
            {
                MotorLK4005.ClearErr(1, 1);
                setDialJamRecoverState(DialJamRecoverState::ClearErrWait);
            }
            break;

        case DialJamRecoverState::ClearErrWait:
            chassis_output.out_dial = 0.0f;
            if (now - g_dial_jam_recover_tick >= kDialJamClearErrWaitMs)
            {
                MotorLK4005.Off(1, 1);
                MotorLK4005.setIsenable(1, false);
                setDialJamRecoverState(DialJamRecoverState::OffWait);
            }
            break;

        case DialJamRecoverState::OffWait:
            chassis_output.out_dial = 0.0f;
            if (now - g_dial_jam_recover_tick >= kDialJamOffWaitMs)
            {
                MotorLK4005.On(1, 1);
                setDialJamRecoverState(DialJamRecoverState::OnWait);
            }
            break;

        case DialJamRecoverState::OnWait:
            chassis_output.out_dial = 0.0f;
            if (now - g_dial_jam_recover_tick >= kDialJamOnWaitMs)
            {
                MotorLK4005.setIsenable(1, true);
                MotorLK4005.setAllowAccumulate(1, true);
                setDialJamRecoverState(DialJamRecoverState::RecoverReverse);
            }
            break;

        case DialJamRecoverState::RecoverReverse:
            chassis_output.out_dial = kDialJamReverseTorque;
            if (now - g_dial_jam_recover_tick >= kDialJamRecoverReverseMs)
            {
                setDialJamRecoverState(DialJamRecoverState::Hold);
            }
            break;

        case DialJamRecoverState::Hold:
            chassis_output.out_dial = 0.0f;
            break;

        default:
            chassis_output.out_dial = 0.0f;
            resetDialJamRecoverState();
            break;
    }

}

/**
 * @brief 鍙戝皠鏈烘瀯涓绘帶鍒跺惊锟?
 * 
 */
void main_loop_launch()
{
    // 鏍规嵁鍙戝皠鐘舵€佹満浣胯兘/澶辫兘鎷ㄧ洏鐢垫満
    static int last_state = -1;

    int state = Cboard.GetLaunchState();
    bool state_changed = (state != last_state);

    if (state == 0)
    {
        resetDialJamRecoverState();
        chassis_target.target_dial = MotorLK4005.getAddAngleDeg(1);
        chassis_output.out_dial = 0.0f;

        if (state_changed)
        {
            MotorLK4005.ClearErr(1, 1);
            MotorLK4005.Off(1, 1);
        }
        MotorLK4005.setIsenable(1, false);

        launch_velocity();
        last_state = state;
        return;
    }

    if (state == 4)
    {
        if (g_dial_jam_recover_state == DialJamRecoverState::Idle)
        {
            setDialJamRecoverState(DialJamRecoverState::Reverse);
        }
    }
    else if (g_dial_jam_recover_state != DialJamRecoverState::Idle)
    {
        if (g_dial_jam_recover_state != DialJamRecoverState::Hold)
        {
            setDialJamRecoverState(DialJamRecoverState::Hold);
        }

        if (!MotorLK4005.getIsenable(1))
        {
            if (state_changed)
            {
                MotorLK4005.On(1, 1);
            }
            MotorLK4005.setIsenable(1, true);
            MotorLK4005.setAllowAccumulate(1, true);
        }

        chassis_target.target_dial = MotorLK4005.getAddAngleDeg(1);
        chassis_output.out_dial = 0.0f;

        if (HAL_GetTick() - g_dial_jam_recover_tick < kDialJamExitHoldMs)
        {
            last_state = state;
            return;
        }

        resetDialJamRecoverState();
    }

    if (!MotorLK4005.getIsenable(1))
    {
        if (state_changed)
        {
            MotorLK4005.On(1, 1);
        }
        MotorLK4005.setIsenable(1, true);
        MotorLK4005.setAllowAccumulate(1, true);
    }
    SetTarget_Launch();

    // 鍩轰簬 Action 锟?Mode 鎵ц鎺у埗鍑芥暟
    switch (state)
    {
        case 1:
            launch_velocity();
            break;
        case 2:
            launch_angle();
            break;
        case 3:
            launch_angle();
            break;
        case 4:
            launch_jam();
            break;
        default:
            launch_velocity();
            break;
    }

    last_state = state;
}
/**
 * @brief 鍗″脊妫€锟?
 * 
 * @return true 
 * @return false 
 */
bool is_jamming()
{
    static uint32_t last_check_time = 0;
    static float last_angle = 0;
    
    float current_angle = Motor3508.getAddAngleDeg(1);
    float err = fabs(chassis_target.target_dial - current_angle);
    
    // 1. 浣嶇疆璇樊瓒呰繃 120 锟?
    if (err > 50.0f)
    {
        uint32_t now = HAL_GetTick();
        
        // 鍒濆锟?
        if (last_check_time == 0)
        {
            last_check_time = now;
            last_angle = current_angle;
            return false;
        }
        
        // 2. 锟?300ms 妫€鏌ヤ竴锟?
        if (now - last_check_time > 300)
        {
            // 3. 濡傛灉锟?300ms 鍐呰浆鍔ㄨ搴﹀皬锟?120 锟?-> 鍒ゅ畾鍗″脊
            if (fabs(current_angle - last_angle) < 50.0f)
            {
                // 閲嶇疆妫€娴嬬姸鎬侊紙鍥犱负鍗冲皢杩涘叆 Jam 妯″紡锛孞am妯″紡锟?error 浼氬彉灏忥級
                last_check_time = 0; 
                return true;
            }
            
            // 鏇存柊鍩哄噯
            last_check_time = now;
            last_angle = current_angle;
        }
    }
    else
    {
        // 璇樊涓嶅ぇ鐨勬椂鍊欙紝閲嶇疆璁℃椂锟?
        last_check_time = 0;
    }
    
    return false;
}


/* 鎺у埗浠诲姟閮ㄥ垎 ------------------------------------------------------------------------------------------------*/

/**
 * @brief 鎺у埗浠诲姟涓诲嚱鏁?
 * 
 * @param argument 浠诲姟鍙傛暟
 */
extern "C"{
void Control(void const * argument)
{
    // 鍒濆鍖栬渹楦ｅ櫒绠＄悊鍣?
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    fsm_init();
    for(;;)
    {
        // 鏇存柊铚傞福鍣ㄧ鐞嗗櫒锛屽鐞嗛槦鍒椾腑鐨勫搷閾冭姹?
        BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().update();
        
        main_loop_gimbal(DT7.get_s1(), DT7.get_s2(), check_online(), alphabet);
        main_loop_launch();
        osDelay(1);
    } 
}
}






