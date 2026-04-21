#ifndef COMMUNICATIONTASK_HPP
#define COMMUNICATIONTASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/HAL/UART/uart_hal.hpp"
#include "../User/core/BSP/RemoteControl/DT7.hpp"
#include "../User/core/BSP/Common/StateWatch/state_watch.hpp"
#include "../User/core/BSP/Common/StateWatch/buzzer_manager.hpp"
#include "../User/core/HAL/CAN/can_hal.hpp"
#include "../User/core/APP/Referee/RM_RefereeSystem.h"
#include <cstdint>

// 鏉块棿閫氳鎺ユ敹缂撳啿鍖哄０鏄?
extern uint8_t BoardRx[64];

typedef struct
{
    uint8_t equipment_online;
    uint8_t change;
    float only_to_auto_time;
    uint8_t is_vision;
    uint8_t is_shoot;
    uint8_t is_jamming;
    uint8_t alphabet[28];
    int32_t launch_state;
} LaunchFSMInput;

/**
 * @brief 鏉块棿閫氳绫?
 * @details 璐熻矗澶勭悊涓庝笂浣嶆満鏉垮瓙涔嬮棿鐨勪覆鍙ｉ€氳锛屽寘鎷繛鎺ョ姸鎬佺洃娴嬨€佹暟鎹В鏋愮瓑鍔熻兘
 */
class BoardCommunication
{
    public:
        /**
         * @brief 鏋勯€犲嚱鏁?
         * @param timeThreshold 杩炴帴瓒呮椂闃堝€?姣)锛岄粯璁?00ms
         */
        BoardCommunication(int timeThreshold = 100) : statewatch_(timeThreshold) 
        {
        }

        /**
         * @brief 铏氭瀽鏋勫嚱鏁?
         */
        virtual ~BoardCommunication() = default;

        /**
         * @brief 鏇存柊鏃堕棿鎴?
         * @details 璁板綍鏈€鍚庝竴娆￠€氳鏃堕棿锛岀敤浜庡垽鏂繛鎺ョ姸鎬?
         */
        void updateTimestamp()
        {
            statewatch_.UpdateLastTime();
        }

        /**
         * @brief 妫€鏌ヨ繛鎺ョ姸鎬?
         * @return true-鍦ㄧ嚎 false-绂荤嚎
         * @details 妫€娴嬮€氳鏄惁姝ｅ父锛岃嫢绂荤嚎鍒欒Е鍙戣渹楦ｅ櫒鎶ヨ
         */
        bool isConnected()
        {
            statewatch_.UpdateTime();
            statewatch_.CheckStatus();
            if(statewatch_.GetStatus() == BSP::WATCH_STATE::Status::OFFLINE)
            {
                BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().requestCommunicationRing();
            }
            return statewatch_.GetStatus() == BSP::WATCH_STATE::Status::ONLINE;
        }
        
        /**
         * @brief 璁剧疆浜戝彴瑙掑害鏁版嵁
         * @param data 鎸囧悜瑙掑害鏁版嵁鐨勬寚閽?4瀛楄妭float)
         */
        void SetYawAngle(uint8_t *data)
        { 
            memcpy(&YawAngal, data, sizeof(float));
        }

        /**
         * @brief 璁剧疆鑳藉惁灏忛檧铻虹姸鎬佹暟鎹?
         * @param data 鎸囧悜鑳藉惁灏忛檧铻虹姸鎬佺殑鎸囬拡(1瀛楄妭bool)
         */
        void SetScroll(uint8_t *data)
        {
            scroll = (*data != 0);
        }

        void SetLaunchFSMInput(uint8_t *data)
        {
            memcpy(&launch_fsm_input.equipment_online, data, sizeof(launch_fsm_input.equipment_online));
            data += sizeof(launch_fsm_input.equipment_online);
            memcpy(&launch_fsm_input.change, data, sizeof(launch_fsm_input.change));
            data += sizeof(launch_fsm_input.change);
            memcpy(&launch_fsm_input.only_to_auto_time, data, sizeof(launch_fsm_input.only_to_auto_time));
            data += sizeof(launch_fsm_input.only_to_auto_time);
            memcpy(&launch_fsm_input.is_vision, data, sizeof(launch_fsm_input.is_vision));
            data += sizeof(launch_fsm_input.is_vision);
            memcpy(&launch_fsm_input.is_shoot, data, sizeof(launch_fsm_input.is_shoot));
            data += sizeof(launch_fsm_input.is_shoot);
            memcpy(&launch_fsm_input.is_jamming, data, sizeof(launch_fsm_input.is_jamming));
            data += sizeof(launch_fsm_input.is_jamming);
            memcpy(launch_fsm_input.alphabet, data, sizeof(launch_fsm_input.alphabet));
            data += sizeof(launch_fsm_input.alphabet);
            memcpy(&launch_fsm_input.launch_state, data, sizeof(launch_fsm_input.launch_state));
        }

        /**
         * @brief 鑾峰彇浜戝彴瑙掑害
         * @return 浜戝彴瑙掑害鍊?寮у害鍗曚綅)
         */
        float GetYawAngle() { return YawAngal; } //寮у害

        /**
         * @brief 鑾峰彇婊氳疆鐘舵€?
         * @return 婊氳疆寮€鍏崇姸鎬?
         */
        bool GetScroll() { return scroll; }

        /**
         * @brief 鑾峰彇鍙戝皠鐘舵€?
         * @return 鍙戝皠鐘舵€?
         */
        const LaunchFSMInput& GetLaunchFSMInput() { return launch_fsm_input; }
        int GetLaunchState() { return launch_fsm_input.launch_state; }
    private:
        BSP::WATCH_STATE::StateWatch statewatch_;  // 杩炴帴鐘舵€佺洃瑙嗗櫒
        float YawAngal;                            // 浜戝彴瑙掑害(寮у害)
        bool scroll;                               // 鑳藉惁灏忛檧铻?true: 涓嶈兘灏忛檧铻?false: 鑳藉皬闄€铻猴紙閬ユ帶鍣ㄦā寮忎笅锛?
        LaunchFSMInput launch_fsm_input{};
    };

/**
 * @brief 瓒呯骇鐢靛鎺у埗绫?
 * @details 璐熻矗瓒呯骇鐢靛鐨凜AN閫氳鎺у埗锛屽寘鎷姸鎬佽鍙栧拰鎸囦护涓嬪彂
 */
class Supercapacitor
{
    public:
        /**
         * @brief 鏋勯€犲嚱鏁?
         * @param id_ 瓒呯骇鐢靛CAN ID
         */
        Supercapacitor(uint32_t id_)
        {
            id = id_;
            last_update_time = 0;
        } 

        /**
         * @brief 妫€鏌ヨ秴绾х數瀹规槸鍚﹀湪绾?
         * @param timeout_ms 瓒呮椂鏃堕棿(ms)
         * @return true 鍦ㄧ嚎
         */
        bool isConnected(uint32_t timeout_ms = 100)
        {
            return (HAL_GetTick() - last_update_time) < timeout_ms;
        } 

        /**
         * @brief 瑙ｆ瀽CAN鎺ユ敹鏁版嵁
         * @param frame CAN鏁版嵁甯?
         * @details 瑙ｆ瀽鏉ヨ嚜瓒呯骇鐢靛鐨勭姸鎬佸弽棣堟暟鎹?
         */
        void Parse(const HAL::CAN::Frame &frame)
        {
            if (frame.id == 0x777)
            {
                const uint8_t* pData = frame.data;
                    
                Power_10times = (float)((int16_t)((pData[0] << 8) | pData[1]));  // 鐢电鍔熺巼(10鍊?
                CurrentEnergy = (float)((int16_t)((pData[2] << 8) | pData[3]));  // 鍓╀綑鑳介噺
                Power         = (float)((int16_t)((pData[4] << 8) | pData[5]));  // 鐢垫満+鏀剧數锛堟鐨勶級鍔熺巼
                state         = pData[6];                                        // 宸ヤ綔鐘舵€?
                cmd           = pData[7];                                        // 褰撳墠鎸囦护

                Power_10times /= 10;    // 杞崲涓哄疄闄呯數绠″姛鐜囧€?      
                Power /= -10;           // 杞崲涓哄疄闄呯數鏈?鏀剧數锛堟鐨勶級鍔熺巼 
                last_update_time = HAL_GetTick();
            }
        }

        /**
         * @brief 鍙戦€佹暟鎹粰瓒呯骇鐢靛
         * @param RatedPower 绛夌骇鍔熺巼
         * @param Instruction 瓒呯數鎸囦护 0:寮€鍚?1:鍏抽棴
         * @param BufferEnergy 缂撳啿鑳介噺
         */
        void sendCAN()
        {
            uint8_t send_data[8] = {0};
            int16_t power_int = (int16_t)(RatedPower);
            send_data[0] = (power_int >> 8) & 0xFF;  // 绛夌骇鍔熺巼楂樺瓧鑺?
            send_data[1] = power_int & 0xFF;         // 绛夌骇鍔熺巼浣庡瓧鑺?

            send_data[2] = Instruction;              // 鎺у埗鎸囦护

            int16_t buffer_int = (int16_t)BufferEnergy;
            send_data[3] = (buffer_int >> 8) & 0xFF; // 缂撳啿鑳介噺楂樺瓧鑺?
            send_data[4] = buffer_int & 0xFF;        // 缂撳啿鑳介噺浣庡瓧鑺?

            send_data[5] = isSupercapOnline;  // 瓒呯數杩炴帴鏍囧織
            send_data[6] = isRefereeOnline;   // 瑁佸垽绯荤粺杩炴帴鏍囧織
            send_data[7] = 0;  // 淇濈暀瀛楄妭
            
            HAL::CAN::Frame frame;
            frame.id = id;                    // CAN ID
            frame.dlc = 8;                    // 鏁版嵁闀垮害
            memcpy(frame.data, send_data, sizeof(send_data));  // 鎷疯礉鏁版嵁
            frame.is_extended_id = false;     // 鏍囧噯甯ф牸寮?
            frame.is_remote_frame = false;    // 鏁版嵁甯?
            
            HAL::CAN::get_can_bus_instance().get_can2().send(frame);  // 鍙戦€丆AN甯?
        }

        /**
         * @brief 璁剧疆绛夌骇鍔熺巼
         * @param power 绛夌骇鍔熺巼鍊?W)
         */
        void setRatedPower(float power)
        {
            RatedPower = power;
        }

        /**
         * @brief 璁剧疆瓒呯數鎸囦护
         * @param instruction 瓒呯數鎸囦护 0:寮€鍚?1:鍏抽棴
         */
        void setInstruction(uint8_t instruction)
        {
            Instruction = instruction;
        }

        /**
         * @brief 璁剧疆缂撳啿鑳介噺
         * @param energy 缂撳啿鑳介噺鍊?J)
         */
        void setBufferEnergy(float energy)
        {
            BufferEnergy = energy;
        }

        void setSupercapOnline(bool online)
        {
            isSupercapOnline = online;
        }

        void setRefereeOnline(bool online)
        {
            isRefereeOnline = online;
        }

        /**
         * @brief 鑾峰彇鐢电鍔熺巼
         * @return 鍔熺巼鍊?W)
         */
        float GetPower_10times() { return Power_10times; }

        /**
         * @brief 鑾峰彇瓒呯骇鐢靛鍓╀綑鑳介噺
         * @return 鍓╀綑鑳介噺鍊?J)
         */
        float GetCurrentEnergy() { return CurrentEnergy; }
        
        /**
         * @brief 鑾峰彇鐢垫満+鏀剧數鍔熺巼
         * @return 鐢垫満鍔熺巼鍊?W)
         */
        float GetPower() { return Power; }
    private:
        float Power_10times;    // 10鍊嶇數绠″姛鐜?
        float CurrentEnergy;    // 瓒呯數鍓╀綑鑳介噺
        float Power;            // 鐢垫満+鏀剧數鍔熺巼
        uint8_t state;          // 鐘舵€?warning锛屾斁鐢碉紱error锛氭煇绉嶉敊璇紝normal锛氭甯?
        uint8_t cmd;            // 褰撳墠鍛戒护

        uint32_t id;            // 瓒呯骇鐢靛ID
        float RatedPower;       // 绛夌骇鍔熺巼
        uint8_t Instruction;    // 瓒呯數鎸囦护 0:寮€鍚?1:鍏抽棴
        float BufferEnergy;     // 缂撳啿鑳介噺
        uint32_t last_update_time; // 涓婃鏇存柊鏃堕棿
        bool isSupercapOnline;
        bool isRefereeOnline;
};

extern Supercapacitor supercap; // 瓒呯數绫诲０鏄?

#endif



