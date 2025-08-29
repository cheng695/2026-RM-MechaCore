/**
 * @file MotorOfflineDetector.cpp
 * @brief 电机掉线检测和蜂鸣器报警功能实现
 */

#include "MotorOfflineDetector.hpp"

/*  =========================== 全局变量的初始化 ===========================  */
MotorStatus_t motor_status[MOTOR_COUNT] = {0};
BuzzerControl_t buzzer_control = {BUZZER_IDLE, 0, 0, 0, {0}, 0, 0, 0};
bool all_motors_connected = true;
/*  =========================== 函数的实现 ===========================  */

/**
 * @brief 初始化电机掉线检测器
 */
void MotorOfflineDetector_Init(void)
{
    // 初始化所有电机状态
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_status[i].last_update_time = 0;
        motor_status[i].is_offline = false;
        motor_status[i].is_alarming = false;
        motor_status[i].alarm_count = 0;
        motor_status[i].alarm_start_time = 0;
        motor_status[i].cycle_start_time = 0;
    }
    
    // 初始化蜂鸣器控制
    buzzer_control.state = BUZZER_IDLE;
    buzzer_control.state_start_time = 0;
    buzzer_control.current_motor_id = 0;
    buzzer_control.current_alarm_count = 0;
    buzzer_control.queue_head = 0;
    buzzer_control.queue_tail = 0;
    buzzer_control.queue_count = 0;
}

/**
 * @brief 更新电机掉线检测器状态
 * @details 该函数需要在主循环中定期调用，建议每10ms调用一次
 */
void MotorOfflineDetector_Update(void)
{
    // 检查电机掉线状态
    MotorOfflineDetector_CheckOffline();
    
    // 处理蜂鸣器报警
    MotorOfflineDetector_HandleBuzzer();
    
    // 检查所有电机连接状态并设置playState
    all_motors_connected = true;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (motor_status[i].is_offline) {
            all_motors_connected = false;
            break;
        }
    }
    
    // 如果所有电机都连接了，设置playState为1，否则为0
    // if (all_motors_connected) {
    //     playState = 1;
    // } else {
    //     playState = 0;
    // }
}

/**
 * @brief 更新指定电机的状态
 * @param motor_id 电机ID
 * @details 当接收到电机数据时调用此函数
 */
void MotorOfflineDetector_UpdateMotorStatus(MotorID_t motor_id)
{
    if (motor_id < MOTOR_COUNT) {
        motor_status[motor_id].last_update_time = HAL_GetTick();
        motor_status[motor_id].is_offline = false;
        
        // 如果电机之前掉线，现在恢复连接，停止报警
        if (motor_status[motor_id].is_alarming) {
            motor_status[motor_id].is_alarming = false;
            motor_status[motor_id].alarm_count = 0;
            
            // 从队列中移除这个电机
            MotorOfflineDetector_RemoveFromQueue(motor_id);
            
            // 如果当前正在为这个电机报警，停止报警
            if (buzzer_control.current_motor_id == motor_id) {
                MotorOfflineDetector_StopAlarm();
            }
        }
    }
}

/**
 * @brief 检查电机掉线状态
 */
void MotorOfflineDetector_CheckOffline(void)
{
    uint32_t current_time = HAL_GetTick();
    
    for (int i = 0; i < MOTOR_COUNT; i++) {
        // 检查是否超过掉线阈值时间
        if (current_time - motor_status[i].last_update_time > OFFLINE_THRESHOLD_MS) {
            if (!motor_status[i].is_offline) {
                // 电机刚刚掉线
                motor_status[i].is_offline = true;
                motor_status[i].is_alarming = true;
                motor_status[i].alarm_start_time = current_time;
                motor_status[i].cycle_start_time = current_time;
                motor_status[i].alarm_count = 0;
                
                // 添加到报警队列
                MotorOfflineDetector_AddToQueue((MotorID_t)i);
            }
        }
    }
}

/**
 * @brief 处理蜂鸣器报警
 */
void MotorOfflineDetector_HandleBuzzer(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed_time = current_time - buzzer_control.state_start_time;
    
    switch (buzzer_control.state) {
        case BUZZER_IDLE:
            // 空闲状态，检查队列中是否有电机需要报警
            if (!MotorOfflineDetector_IsQueueEmpty()) {
                uint8_t next_motor = MotorOfflineDetector_GetNextMotor();
                buzzer_control.current_motor_id = next_motor;
                buzzer_control.current_alarm_count = 0;
                buzzer_control.state = BUZZER_ON;
                buzzer_control.state_start_time = HAL_GetTick();
                
                // 开启蜂鸣器
                uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
                uint32_t arr = timer_freq / 523; // 523Hz = 中音1
                __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5); // 20%占空比
                __HAL_TIM_SetCounter(&htim12, 0);
            }
            break;
            
        case BUZZER_ON:
            // 蜂鸣器开启状态
            if (elapsed_time >= BUZZER_ON_TIME_MS) {
                // 关闭蜂鸣器
                __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
                buzzer_control.state = BUZZER_OFF;
                buzzer_control.state_start_time = current_time;
            }
            break;
            
        case BUZZER_OFF:
            // 蜂鸣器关闭状态
            if (elapsed_time >= BUZZER_OFF_TIME_MS) {
                buzzer_control.current_alarm_count++;
                
                // 检查是否完成当前电机的所有报警次数
                uint8_t target_alarm_count = buzzer_control.current_motor_id + 1;
                if (buzzer_control.current_alarm_count >= target_alarm_count) {
                    // 完成一轮报警，检查队列中是否还有其他电机
                    if (buzzer_control.queue_count > 0) {
                        // 还有电机在队列中，进入电机间等待状态
                        buzzer_control.state = BUZZER_MOTOR_WAIT;
                        buzzer_control.state_start_time = current_time;
                    } else {
                        // 队列为空，进入周期等待
                        buzzer_control.state = BUZZER_CYCLE_WAIT;
                        buzzer_control.state_start_time = current_time;
                    }
                    
                    // 更新电机报警状态
                    if (buzzer_control.current_motor_id < MOTOR_COUNT) {
                        motor_status[buzzer_control.current_motor_id].alarm_count++;
                    }
                } else {
                    // 继续下一声报警
                    uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
                    uint32_t arr = timer_freq / 523; // 523Hz = 中音1
                    __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
                    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5); // 20%占空比
                    __HAL_TIM_SetCounter(&htim12, 0);
                    buzzer_control.state = BUZZER_ON;
                    buzzer_control.state_start_time = current_time;
                }
            }
            break;
            
        case BUZZER_CYCLE_WAIT:
            // 周期等待状态
            if (elapsed_time >= BUZZER_CYCLE_WAIT_MS) {
                // 检查当前电机是否仍然掉线
                if (buzzer_control.current_motor_id < MOTOR_COUNT) {
                    if (motor_status[buzzer_control.current_motor_id].is_offline) {
                        // 电机仍然掉线，重新构建队列并开始新一轮报警
                        // 清空当前队列
                        buzzer_control.queue_head = 0;
                        buzzer_control.queue_tail = 0;
                        buzzer_control.queue_count = 0;
                        
                        // 重新添加所有掉线的电机到队列
                        for (int i = 0; i < MOTOR_COUNT; i++) {
                            if (motor_status[i].is_offline && motor_status[i].is_alarming) {
                                MotorOfflineDetector_AddToQueue((MotorID_t)i);
                            }
                        }
                        
                        // 开始新一轮报警
                        if (!MotorOfflineDetector_IsQueueEmpty()) {
                            uint8_t next_motor = MotorOfflineDetector_GetNextMotor();
                            buzzer_control.current_motor_id = next_motor;
                            buzzer_control.current_alarm_count = 0;
                            buzzer_control.state = BUZZER_ON;
                            buzzer_control.state_start_time = current_time;
                            
                            // 开启蜂鸣器
                            uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
                            uint32_t arr = timer_freq / 523; // 523Hz = 中音1
                            __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
                            __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5); // 20%占空比
                            __HAL_TIM_SetCounter(&htim12, 0);
                        } else {
                            buzzer_control.state = BUZZER_IDLE;
                        }
                    } else {
                        // 电机已恢复连接，从队列中移除并检查下一个电机
                        MotorOfflineDetector_RemoveFromQueue((MotorID_t)buzzer_control.current_motor_id);
                        motor_status[buzzer_control.current_motor_id].is_alarming = false;
                        
                        // 检查队列中是否还有其他电机
                        if (!MotorOfflineDetector_IsQueueEmpty()) {
                            buzzer_control.state = BUZZER_MOTOR_WAIT;
                            buzzer_control.state_start_time = current_time;
                        } else {
                            buzzer_control.state = BUZZER_IDLE;
                        }
                    }
                } else {
                    buzzer_control.state = BUZZER_IDLE;
                }
            }
            break;
            
        case BUZZER_MOTOR_WAIT:
            // 电机间等待状态（等待2秒后切换到下一个电机）
            if (elapsed_time >= BUZZER_CYCLE_WAIT_MS) {
                // 获取下一个电机并开始报警
                if (!MotorOfflineDetector_IsQueueEmpty()) {
                    uint8_t next_motor = MotorOfflineDetector_GetNextMotor();
                    buzzer_control.current_motor_id = next_motor;
                    buzzer_control.current_alarm_count = 0;
                    buzzer_control.state = BUZZER_ON;
                    buzzer_control.state_start_time = current_time;
                    
                    // 开启蜂鸣器
                    uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
                    uint32_t arr = timer_freq / 523; // 523Hz = 中音1
                    __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
                    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5); // 20%占空比
                    __HAL_TIM_SetCounter(&htim12, 0);
                } else {
                    buzzer_control.state = BUZZER_IDLE;
                }
            }
            break;
    }
}

/**
 * @brief 开始电机掉线报警
 * @param motor_id 掉线的电机ID
 */
void MotorOfflineDetector_StartAlarm(MotorID_t motor_id)
{
    // 这个函数现在主要用于外部调用，内部逻辑已经移到状态机中
    if (motor_id < MOTOR_COUNT && buzzer_control.state == BUZZER_IDLE) {
        buzzer_control.current_motor_id = motor_id;
        buzzer_control.current_alarm_count = 0;
        buzzer_control.state = BUZZER_ON;
        buzzer_control.state_start_time = HAL_GetTick();
        
        // 开启蜂鸣器 - 使用中音1的频率
        uint32_t timer_freq = TIM_GetCounterFreq(&htim12);
        uint32_t arr = timer_freq / 523; // 523Hz = 中音1
        __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5); // 20%占空比
        __HAL_TIM_SetCounter(&htim12, 0);
    }
}

/**
 * @brief 停止电机掉线报警
 */
void MotorOfflineDetector_StopAlarm(void)
{
    // 关闭蜂鸣器
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    
    // 重置蜂鸣器控制状态
    buzzer_control.state = BUZZER_IDLE;
    buzzer_control.current_motor_id = 0;
    buzzer_control.current_alarm_count = 0;
}

/**
 * @brief 添加电机到报警队列
 * @param motor_id 电机ID
 */
void MotorOfflineDetector_AddToQueue(MotorID_t motor_id)
{
    if (motor_id < MOTOR_COUNT && buzzer_control.queue_count < MOTOR_COUNT) {
        // 检查电机是否已经在队列中
        for (int i = 0; i < buzzer_control.queue_count; i++) {
            uint8_t index = (buzzer_control.queue_head + i) % MOTOR_COUNT;
            if (buzzer_control.motor_queue[index] == motor_id) {
                return; // 电机已在队列中
            }
        }
        
        // 添加到队列尾部
        buzzer_control.motor_queue[buzzer_control.queue_tail] = motor_id;
        buzzer_control.queue_tail = (buzzer_control.queue_tail + 1) % MOTOR_COUNT;
        buzzer_control.queue_count++;
    }
}

/**
 * @brief 从报警队列中移除电机
 * @param motor_id 电机ID
 */
void MotorOfflineDetector_RemoveFromQueue(MotorID_t motor_id)
{
    if (motor_id < MOTOR_COUNT && buzzer_control.queue_count > 0) {
        // 查找电机在队列中的位置
        for (int i = 0; i < buzzer_control.queue_count; i++) {
            uint8_t index = (buzzer_control.queue_head + i) % MOTOR_COUNT;
            if (buzzer_control.motor_queue[index] == motor_id) {
                // 移除电机，将后面的电机前移
                for (int j = i; j < buzzer_control.queue_count - 1; j++) {
                    uint8_t current_index = (buzzer_control.queue_head + j) % MOTOR_COUNT;
                    uint8_t next_index = (buzzer_control.queue_head + j + 1) % MOTOR_COUNT;
                    buzzer_control.motor_queue[current_index] = buzzer_control.motor_queue[next_index];
                }
                buzzer_control.queue_count--;
                buzzer_control.queue_tail = (buzzer_control.queue_tail - 1 + MOTOR_COUNT) % MOTOR_COUNT;
                break;
            }
        }
    }
}

/**
 * @brief 获取队列中的下一个电机
 * @return 下一个电机的ID
 */
uint8_t MotorOfflineDetector_GetNextMotor(void)
{
    if (buzzer_control.queue_count > 0) {
        uint8_t motor_id = buzzer_control.motor_queue[buzzer_control.queue_head];
        buzzer_control.queue_head = (buzzer_control.queue_head + 1) % MOTOR_COUNT;
        buzzer_control.queue_count--;
        return motor_id;
    }
    return 0xFF; // 无效ID
}

/**
 * @brief 检查队列是否为空
 * @return true表示队列为空，false表示队列不为空
 */
bool MotorOfflineDetector_IsQueueEmpty(void)
{
    return buzzer_control.queue_count == 0;
}
