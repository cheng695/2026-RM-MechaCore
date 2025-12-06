#ifndef BUZZER_MANAGER_HPP
#define BUZZER_MANAGER_HPP

#include "cmsis_os.h"
#include "tim.h"  // 用于蜂鸣器控制
#include <stdint.h>

namespace BSP::WATCH_STATE
{
    class BuzzerManagerSimple
    {
    public:
        // 单例访问
        static BuzzerManagerSimple& getInstance();
        
        // 初始化
        void init();
        
        // 请求电机响铃
        void requestMotorRing(uint8_t motor_id);
        
        // 请求遥控器响铃  
        void requestRemoteRing();
        
        // 更新处理（需要在主循环中定期调用）
        void update();
        
    private:
        // 私有构造函数，确保单例
        BuzzerManagerSimple();
        
        // 实际的响铃处理
        void processRing(uint8_t id);
        
        // 实际的蜂鸣器控制函数
        void controlBuzzer(bool on_off);
        
        // 队列相关
        static constexpr uint8_t MAX_QUEUE_SIZE = 10;
        uint8_t ring_queue_[MAX_QUEUE_SIZE];
        uint8_t ring_index_ = 0;
        
        // 时间控制
        uint32_t last_ring_time_ = 0;
        bool is_ringing_ = false;
        static constexpr uint32_t RING_INTERVAL_MS = 500;  // 响铃间隔
        
        // 蜂鸣器参数
        static constexpr uint16_t BUZZER_PWM_VALUE = 200;
        static constexpr uint16_t SHORT_BEEP_MS = 100;
        static constexpr uint16_t LONG_BEEP_MS = 500;
        static constexpr uint16_t BETWEEN_BEEP_MS = 100;
        static constexpr uint16_t AFTER_BEEP_MS = 200;
    };
}

#endif
