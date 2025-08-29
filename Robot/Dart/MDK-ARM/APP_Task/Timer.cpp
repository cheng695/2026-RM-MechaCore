#include "Inhpp.hpp"
#include "MotorOfflineDetector.hpp"
#include "timers.h"

/*  =========================== 全局变量的初始化 ===========================  */
// int Yaw_Count          = 0; // yaw轴计数器
// int TrafficLight_Count = 0; // 交通灯计数器
// 定义全局变量和定时器句柄
TimerHandle_t xNoteTimer;    // 用于音符延时的定时器
TimerHandle_t xRestTimer;    // 用于静音间隔的定时器
/*  ==============================进程的变量===============================  */
// TickType_t SystemTick; // 系统滴答计数
uint16_t psc = 0;
uint16_t pwm = MIN_BUZZER_PWM;

// 播放状态
uint8_t playState = 1;
// 播放进度
uint32_t playIndex = 0;
// 节拍速度(每分钟多少拍) 
uint16_t bpm = 171;
uint32_t timFrequency_;
float noteDuration;
int Song_Length = 0;
/*  =========================== 函数的声明 ===========================  */
void CheckSystemStatus_TrafficLight();
void SyncYawAngle();
void Buzzer_Timers_Init(void);
void NoteTimerCallback(TimerHandle_t xTimer);
void RestTimerCallback(TimerHandle_t xTimer);
void Handle_Next_Note(void);

void TimerCallback(void *argument)
{
    /* USER CODE BEGIN LED_Flashing */
    TickType_t Lasttick = xTaskGetTickCount();
    // 开始PWM输出
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    // TIM4的计数频率
    timFrequency_ = TIM_GetCounterFreq(&htim12);
    // 每拍的持续时间
    noteDuration = 1000 * 60 / bpm;
    
    // 初始化电机掉线检测器
    MotorOfflineDetector_Init();
    Buzzer_Timers_Init();
    vTaskDelay(3000); // 延时，确保定时器初始化完成
    /* Infinite loop */
    for (;;) {

        //uint32_t SystemTick = HAL_GetTick(); // 获取系统滴答数

        // 优先处理电机断连报警（每10毫秒执行一次）
        if (SystemTick % 10 == 0) { 
            SyncYawAngle();         // 同步Yaw轴角度
        }

        if (SystemTick % 300 == 0) {
            CheckSystemStatus_TrafficLight();
        }

        MotorOfflineDetector_Update(); // 更新电机掉线检测器状态
        Handle_Buzzer(); // 处理蜂鸣器播放

        vTaskDelayUntil(&Lasttick, pdMS_TO_TICKS(1)); // 每1毫秒执行一次
    }
    /* USER CODE END LED_Flashing */
}

/* Private application code --------------------------------------------------*/
void CheckSystemStatus_TrafficLight()
{
    if (DT7UartCom.rc.s1 == DOWN && DT7UartCom.rc.s2 == DOWN)
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14); // 绿灯闪烁
    else if (DT7UartCom.isConnected == true)
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET); // 绿灯常亮
    else
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_SET); // 绿灯关闭

    if (MyRefereeSystemData.SOF == 0xA5)
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET); // 红灯常亮
    else
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11); // 红灯闪烁

    osDelay(300); // 延时300毫秒
}

void SyncYawAngle()
{
    if ((DT7UartCom.rc.s1 == MID) && DT7UartCom.isConnected == true && DT7UartCom.Coord.ch0 != 0) {
        // yaw轴的值加等于摇杆的值
        Dart.Add_Motor_Target(&Dart.Yaw_Angle, DT7UartCom.Coord.ch0 * -0.02, LEAST_YAW, MAX_YAW);
    }
}

// void Handle_Buzzer(void)
// {
//     // 检查是否有电机掉线报警，如果有则不播放音乐
//     // if (all_motors_connected == false) {
//     //     playState = 0;
//     //     return; // 有电机掉线报警，不播放音乐
//     // }
    
//     Song_Length = sizeof(You) / sizeof(Bate);
//     if (playState){
//         const Bate bate = You[playIndex];
//         if (bate.frequency == P0) {
//           // 休止符
//           __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
//         } else {
//           // 将频率转换为计数值, 设置到自动重装载寄存器
//           uint32_t arr = timFrequency_ / bate.frequency;
//           __HAL_TIM_SET_AUTORELOAD(&htim12,arr);
//           // 设置占空比为20%
//           __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5); // 20%占空比
//           // 从0开始计数 重置PWM波形
//           __HAL_TIM_SetCounter(&htim12, 0);
//         }
//         // 延时该音符的持续时间 (5ms的空白以区分连续两个相同的音符)
//         // vTaskDelay(pdMS_TO_TICKS((uint32_t)(bate.period * noteDuration) - 5));
//         // __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
//         // vTaskDelay(pdMS_TO_TICKS(5));
//         HAL_Delay((uint32_t) (bate.period * noteDuration) - 5);
//         __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
//         HAL_Delay(5);
  
//         // 下一个音符
//         playIndex++;
//         // 播放结束
//         if (playIndex >= Song_Length){
//           playState = 0;
//           playIndex = 0;
//         }
//     }
// }

/**
 * 计算定时器计数频率
 */
uint32_t TIM_GetCounterFreq(TIM_HandleTypeDef *htim)
{
    uint32_t timer_clock;
    // 高级定时器是APB2
    if (htim->Instance == TIM1) {
        timer_clock = HAL_RCC_GetPCLK2Freq();
        // 如果APB分频不为1，定时器时钟会翻倍
        if (HAL_RCC_GetPCLK2Freq() != (HAL_RCC_GetHCLKFreq() / 1)) {
            timer_clock *= 2;
        }
    } else {
        // 其他定时器是APB1
        timer_clock = HAL_RCC_GetPCLK1Freq();
        // 如果APB分频不为1，定时器时钟会翻倍
        if (HAL_RCC_GetPCLK1Freq() != (HAL_RCC_GetHCLKFreq() / 1)) {
            timer_clock *= 2;
        }
    }

    uint32_t prescaler = htim->Instance->PSC;
    return timer_clock / (prescaler + 1);
}


// 初始化定时器（在程序初始化时调用）
void Buzzer_Timers_Init(void)
{
    xNoteTimer = xTimerCreate("NoteTimer", pdMS_TO_TICKS(1), pdFALSE, NULL, NoteTimerCallback);
    xRestTimer = xTimerCreate("RestTimer", pdMS_TO_TICKS(1), pdFALSE, NULL, RestTimerCallback);
    
    // 检查定时器是否创建成功
    if (xNoteTimer == NULL || xRestTimer == NULL) {
        // 定时器创建失败处理
        Error_Handler();
    }
}

// 音符定时器回调函数
void NoteTimerCallback(TimerHandle_t xTimer)
{
    // 停止当前音符（静音）
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
    // 启动静音定时器（5ms）
    xTimerChangePeriod(xRestTimer, pdMS_TO_TICKS(5), portMAX_DELAY);
    xTimerStart(xRestTimer, portMAX_DELAY);
}

// 静音定时器回调函数
void RestTimerCallback(TimerHandle_t xTimer)
{
    // 播放下一个音符
    playIndex++;
    if (playIndex >= Song_Length) {
        playState = 0;
        playIndex = 0;
        // 停止定时器
        xTimerStop(xNoteTimer, 0);
        xTimerStop(xRestTimer, 0);
    } else {
        // 直接处理下一个音符（避免任务调度延迟）
        Handle_Next_Note();
    }
}

// 处理下一个音符（无延时）
void Handle_Next_Note(void)
{
    const Bate bate = Xixirangrang[playIndex];
    if (bate.frequency == P0) {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 0);
        // 对于休止符，也启动定时器来处理时长
        xTimerChangePeriod(xNoteTimer, pdMS_TO_TICKS(bate.period * noteDuration - 5), portMAX_DELAY);
        xTimerStart(xNoteTimer, portMAX_DELAY);
    } else {
        uint32_t arr = timFrequency_ / bate.frequency;
        __HAL_TIM_SET_AUTORELOAD(&htim12, arr);
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr / 5);
        __HAL_TIM_SetCounter(&htim12, 0);
        // 启动音符定时器（减去5ms的静音间隔）
        xTimerChangePeriod(xNoteTimer, pdMS_TO_TICKS(bate.period * noteDuration - 5), portMAX_DELAY);
        xTimerStart(xNoteTimer, portMAX_DELAY);
    }
}

// 主处理函数（在任务中循环调用）
void Handle_Buzzer(void)
{
    // 检查是否有电机掉线报警，如果有则不播放音乐
    if (all_motors_connected == false) {
        playState = 0;
        // 停止音乐播放定时器
        if (xTimerIsTimerActive(xNoteTimer)) {
            xTimerStop(xNoteTimer, 0);
        }
        if (xTimerIsTimerActive(xRestTimer)) {
            xTimerStop(xRestTimer, 0);
        }
        return; // 有电机掉线报警，不播放音乐
    }
    
    Song_Length = sizeof(Xixirangrang) / sizeof(Bate);
    if (playState && !xTimerIsTimerActive(xNoteTimer) && !xTimerIsTimerActive(xRestTimer)) {
        Handle_Next_Note();
    }
}
