#include "buzzer_manager.hpp"

namespace BSP::WATCH_STATE
{
    BuzzerManagerSimple& BuzzerManagerSimple::getInstance()
    {
        static BuzzerManagerSimple instance;
        return instance;
    }
    
    BuzzerManagerSimple::BuzzerManagerSimple()
    {
        // 构造函数
    }
    
    void BuzzerManagerSimple::init()
    {
        ring_index_ = 0;
        last_ring_time_ = 0;
        is_ringing_ = false;
    }
    
    void BuzzerManagerSimple::requestMotorRing(uint8_t motor_id)
    {
        if (motor_id < 1 || motor_id > 4)
            return;  // 无效的电机ID
            
        // 检查队列是否已满
        if (ring_index_ >= MAX_QUEUE_SIZE)
            return;
            
        // 检查是否已在队列中（避免重复）
        for (uint8_t i = 0; i < ring_index_; i++)
        {
            if (ring_queue_[i] == motor_id)
                return;  // 已在队列中
        }
        
        // 添加到队列
        ring_queue_[ring_index_++] = motor_id;
    }
    
    void BuzzerManagerSimple::requestRemoteRing()
    {
        // 使用特殊ID 0xFF 表示遥控器
        uint8_t remote_id = 0xFF;
        
        // 检查队列是否已满
        if (ring_index_ >= MAX_QUEUE_SIZE)
            return;
            
        // 检查是否已在队列中
        for (uint8_t i = 0; i < ring_index_; i++)
        {
            if (ring_queue_[i] == remote_id)
                return;
        }
        
        // 添加到队列
        ring_queue_[ring_index_++] = remote_id;
    }
    
    void BuzzerManagerSimple::update()
    {
        uint32_t current_time = HAL_GetTick();
        
        // 如果正在响铃，跳过
        if (is_ringing_)
            return;
            
        // 如果距离上次响铃时间太短，跳过
        if (current_time - last_ring_time_ < RING_INTERVAL_MS)
            return;
            
        // 如果队列中有请求，处理第一个
        if (ring_index_ > 0)
        {
            uint8_t id = ring_queue_[0];
            
            // 从队列中移除第一个元素
            for (uint8_t i = 1; i < ring_index_; i++)
            {
                ring_queue_[i-1] = ring_queue_[i];
            }
            ring_index_--;
            
            // 标记为正在响铃
            is_ringing_ = true;
            
            // 执行响铃
            processRing(id);
            
            // 更新上次响铃时间
            last_ring_time_ = HAL_GetTick();
            
            // 标记响铃结束
            is_ringing_ = false;
        }
    }
    
    void BuzzerManagerSimple::processRing(uint8_t id)
    {
        if (id == 0xFF)  // 遥控器
        {
            // 长响一声
            controlBuzzer(true);
            osDelay(LONG_BEEP_MS);
            controlBuzzer(false);
            osDelay(LONG_BEEP_MS);
        }
        else  // 电机，id=1-4
        {
            // 根据电机ID响对应次数
            for (uint8_t i = 0; i < id; i++)
            {
                controlBuzzer(true);
                osDelay(SHORT_BEEP_MS);
                controlBuzzer(false);
                
                // 如果不是最后一次，添加间隔
                if (i < id - 1)
                    osDelay(BETWEEN_BEEP_MS);
            }
            // 所有响铃结束后稍作停顿
            osDelay(AFTER_BEEP_MS);
        }
    }
    
    void BuzzerManagerSimple::controlBuzzer(bool on_off)
    {
        if (on_off)
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, BUZZER_PWM_VALUE);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
        }
    }
}
