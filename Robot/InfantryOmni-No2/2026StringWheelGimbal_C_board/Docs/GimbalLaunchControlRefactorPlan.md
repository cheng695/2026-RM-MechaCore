# 云台与发射机构控制层重构方案

## 1. 改造目标

当前 `ControlTask.cpp` 同时承担以下职责：

- 创建和保存状态机、PID、滤波器、电机对象；
- 解析云台和发射机构状态；
- 生成云台目标角度、速度和 Pitch 折叠目标；
- 生成拨盘和摩擦轮目标；
- 执行 Yaw、Pitch、拨盘和摩擦轮 PID；
- 处理卡弹恢复；
- 向 `MotorTask` 提供最终输出。

这样会导致控制任务过长，而且状态判断、目标规划、电机控制互相耦合。

目标结构：

```text
SerialTask
    ↓
OperatorInput
    ├── BuildGimbalInput() -> GimbalInput
    └── BuildLaunchInput() -> LaunchInput
            ↓
    ┌───────────────────────┐
    │ GimbalEventDetector   │
    │ LaunchEventDetector   │
    └───────────────────────┘
            ↓
    ┌───────────────────────┐
    │ GimbalStateMachine    │
    │ LaunchStateMachine    │
    └───────────────────────┘
            ↓
    ┌───────────────────────┐
    │ GimbalPlanner         │
    │ LaunchPlanner         │
    └───────────────────────┘
            ↓
    RobotContext
            ↓
    GimbalController / LaunchController
            ↓
    MotorTask
```

## 2. 各模块职责

### 2.1 SerialTask

只负责：

- 接收 DT7、IMU 和视觉数据；
- 更新按键对象；
- 产生按下沿、释放沿；
- 更新公共 `OperatorInput`；
- 保存 B、V 等需要跨周期保持的输入状态。

不负责：

- 判断云台状态；
- 判断发射状态；
- 计算电机目标；
- 直接调用 PID；
- 直接控制电机。

### 2.2 OperatorInput

`OperatorInput` 是串口层产生的公共输入快照。

当前阶段可以保留：

- 遥控器左右开关；
- 键盘/鼠标状态；
- `V` 折叠键；
- `B` 发射使能；
- `Z/X` 单发/连发选择；
- 鼠标左键按下沿和释放信息；
- 视觉标志和视觉发射模式。

后续由两个 Builder 分别提取：

```cpp
GimbalInput BuildGimbalInput(const OperatorInput& input);
LaunchInput BuildLaunchInput(const OperatorInput& input,
                             bool equipment_online,
                             bool jam_detected);
```

### 2.3 GimbalEventDetector

负责把当前云台输入转换成：

```text
StopRequested
DownRequested
ManualRequested
VisionRequested
```

它不执行：

- Yaw 对齐；
- Pitch 回正；
- VMC；
- 电机控制。

### 2.4 LaunchEventDetector

负责把 `LaunchInput` 转换成：

```text
StopRequested
CeasefireRequested
OnlyRequested
AutoRequested
JamDetected
```

其中：

- `ONLY` 是持续模式；
- 单发一发的动作不是状态切换；
- `JAM` 是临时安全状态；
- `STOP` 优先级高于 `JAM`。

## 3. RobotContext 设计

### 3.1 RobotContext 的职责

`RobotContext` 保存跨任务共享的运行数据：

```cpp
extern Gimbal_FSM gimbal_fsm;
extern Launch_FSM launch_fsm;

extern GimbalTarget gimbal_target;
extern LaunchTarget launch_target;

extern Output_gimbal gimbal_output;
extern Output_launch launch_output;
```

如果项目仍然需要统一目标结构，也可以暂时保留：

```cpp
extern ControlTask gimbal_target;
```

但最终应该改名，因为 `ControlTask` 是任务名，不应该同时作为目标数据类型。

### 3.2 RobotContext 不应该保存的内容

以下内容不建议放入 RobotContext：

- Yaw PID；
- Pitch PID；
- VMC 对象；
- 滤波器；
- 发射拨盘 PID；
- 摩擦轮 PID；
- 具体控制算法的临时变量。

这些属于对应的 Planner 或 Controller。RobotContext 只提供共享数据，不承载所有控制算法。

### 3.3 建议文件

```text
User/Module/RobotContext.hpp
User/Module/RobotContext.cpp
```

`RobotContext.cpp` 负责定义共享对象，其他模块通过 `RobotContext.hpp` 使用它们，避免各个 Task 互相 include。

## 4. GimbalPlanner 设计

建议新增：

```text
User/Module/Gimbal/GimbalPlanner.hpp
User/Module/Gimbal/GimbalPlanner.cpp
```

### 4.1 GimbalPlanner 负责的内容

将当前 `ControlTask.cpp` 中的 `SetTarget_Gimbal()` 拆入 Planner。

主要包括：

- STOP 状态目标保持；
- MANUAL 状态遥控器目标；
- 键鼠模式 Yaw/Pitch 目标；
- VISION 状态视觉目标；
- DOWN/TRANSFORM 状态的 Yaw 目标；
- Yaw 到位前锁定 Pitch 当前反馈；
- Yaw 到位后释放 Pitch 折叠目标；
- Pitch1/Pitch2 目标限幅；
- 状态切换时滤波器重置；
- 当前姿态保持目标。

建议接口：

```cpp
void InitGimbalPlanner();

GimbalTarget PlanGimbalTarget(Enum_Gimbal_States state,
                              const GimbalInput& input);
```

如果需要读取 IMU、视觉或电机反馈，可以通过参数传入反馈，而不是在 Planner 中直接到处访问全局对象：

```cpp
struct GimbalFeedback
{
    float yaw_angle_deg;
    float pitch1_angle_deg;
    float pitch2_angle_deg;
    float yaw_gyro;
    float pitch2_gyro;
};
```

更完整的接口：

```cpp
GimbalTarget PlanGimbalTarget(Enum_Gimbal_States state,
                              const GimbalInput& input,
                              const GimbalFeedback& feedback);
```

### 4.2 云台折叠过程

折叠不是一个普通的目标角度切换，应保留过程保护：

```text
进入 DOWN
    ↓
Yaw 转到折叠目标角度
    ↓
Yaw 未到位：Pitch 目标保持当前反馈
    ↓
Yaw 到位：释放 Pitch 折叠目标
    ↓
Pitch 完成折叠
    ↓
进入 DOWN 稳定状态
```

因此：

- `GimbalStateMachine` 负责记录 `ManualToDown`、`VisionToDown` 等转换；
- `GimbalPlanner` 负责生成折叠过程目标；
- `GimbalController` 负责真正执行 Yaw/Pitch 控制；
- 不能在事件检测器中直接驱动电机。

回正过程同理：

```text
DOWN -> MANUAL
    ↓
保持 Yaw 当前角度
    ↓
Pitch 回到正常位置
    ↓
回正完成后允许 MANUAL 控制
```

如果当前 `GimbalStateMachine` 只有 `CompleteTransition()`，建议由 Controller 判断动作完成，再调用：

```cpp
gimbal_fsm.CompleteTransition();
```

### 4.3 GimbalController

当前的以下函数属于 Controller，而不是 Planner：

```text
gimbal_stop()
gimbal_manual()
gimbal_vision()
gimbal_transform()
```

建议后续拆为：

```text
User/Module/Gimbal/GimbalController.hpp
User/Module/Gimbal/GimbalController.cpp
```

接口示例：

```cpp
void InitGimbalController();

GimbalOutput ControlGimbal(const GimbalTarget& target,
                           const GimbalFeedback& feedback,
                           Enum_Gimbal_States state);
```

Controller 内部保存：

- Yaw ADRC/PID；
- Pitch ADRC/PID；
- VMC；
- 重力补偿；
- 摩擦补偿；
- 控制滤波器。

这样 Planner 不需要知道 PID 的具体实现。

## 5. LaunchPlanner 设计

建议新增：

```text
User/Module/Launch/LaunchPlanner.hpp
User/Module/Launch/LaunchPlanner.cpp
```

### 5.1 LaunchPlanner 负责的内容

将当前 `SetTarget_Launch()` 中的目标生成逻辑迁移到这里：

- STOP 状态目标；
- CEASEFIRE 状态目标；
- ONLY 状态目标；
- AUTO 状态目标；
- JAM 状态目标；
- 摩擦轮目标速度；
- 拨盘目标角度/速度；
- 单发拨盘角度增量；
- 鼠标左键、拨轮边沿产生的单发动作。

建议接口：

```cpp
void InitLaunchPlanner();

LaunchTarget PlanLaunchTarget(Enum_Launch_States state,
                               const LaunchInput& input,
                               const LaunchFeedback& feedback);
```

### 5.2 发射状态与发射动作分离

状态机只管理工作模式：

```text
LAUNCH_STOP
LAUNCH_CEASEFIRE
LAUNCH_ONLY
LAUNCH_AUTO
LAUNCH_JAM
```

单发动作单独处理：

```text
SingleShotRequested
    ↓
拨盘目标增加一发弹丸对应角度
    ↓
状态仍保持 LAUNCH_ONLY
```

不能再使用：

```cpp
is_shoot -> LAUNCH_CEASEFIRE
```

否则单发状态会被错误退出。

### 5.3 ONLY 超时

旧逻辑中的：

```text
ONLY 状态运行超过 4000 个 200 Hz 周期
    ↓
自动进入 AUTO
```

应保留在 `LaunchStateMachine::TIM_Update()` 或内部超时事件中。

它与是否已经发射一发弹丸无关。

### 5.4 鼠标左键逻辑

保留旧行为：

```text
鼠标左键按下
    ├─ 当前/上一个状态为 ONLY，且此前释放过
    │      └─ 保持 ONLY
    └─ 其他情况
           └─ 进入 AUTO
```

建议：

- `LaunchInput` 提供 `mouse_left_pressed`；
- `LaunchInput` 提供 `mouse_left_released` 或释放锁存结果；
- `LaunchEventDetector` 只负责判断请求；
- `LaunchPlanner` 负责单发拨盘目标。

### 5.5 LaunchController

当前以下函数属于 Controller：

```text
launch_velocity()
launch_angle()
launch_jam()
```

建议拆为：

```text
User/Module/Launch/LaunchController.hpp
User/Module/Launch/LaunchController.cpp
```

职责：

| 状态 | Controller 行为 |
|---|---|
| `LAUNCH_STOP` | 摩擦轮和拨盘输出归零 |
| `LAUNCH_CEASEFIRE` | 摩擦轮预旋，拨盘停止 |
| `LAUNCH_ONLY` | 摩擦轮预旋，拨盘位置闭环 |
| `LAUNCH_AUTO` | 摩擦轮预旋，拨盘连续送弹 |
| `LAUNCH_JAM` | 执行拨盘反转退弹 |

如果拨盘电机需要执行清错、掉电、重新上电等流程，这部分应单独抽成：

```text
DialJamRecovery
```

不要把完整的电机重启流程堆进状态机。

## 6. ControlTask 最终形态

重构完成后，`ControlTask.cpp` 只负责调度：

```cpp
void ControlStep()
{
    const bool online = check_online();

    const OperatorInput operator_input = BuildOperatorInput();

    const GimbalInput gimbal_input =
        BuildGimbalInput(operator_input);

    const LaunchInput launch_input =
        BuildLaunchInput(operator_input,
                         online,
                         IsJamming());

    gimbal_fsm.Dispatch(
        DetectGimbalEvent(gimbal_input, online));

    launch_fsm.TIM_Update();
    launch_fsm.Dispatch(
        DetectLaunchEvent(launch_input,
                          launch_fsm.Get_Now_State()));

    const GimbalFeedback gimbal_feedback =
        ReadGimbalFeedback();

    const LaunchFeedback launch_feedback =
        ReadLaunchFeedback();

    gimbal_target =
        PlanGimbalTarget(gimbal_fsm.Get_Now_State(),
                         gimbal_input,
                         gimbal_feedback);

    launch_target =
        PlanLaunchTarget(launch_fsm.Get_Now_State(),
                         launch_input,
                         launch_feedback);

    gimbal_output =
        ControlGimbal(gimbal_target,
                      gimbal_feedback,
                      gimbal_fsm.Get_Now_State());

    launch_output =
        ControlLaunch(launch_target,
                      launch_feedback,
                      launch_fsm.Get_Now_State());
}
```

控制任务不再：

- 直接判断挡位组合；
- 直接读取 `alphabet`；
- 直接生成云台目标；
- 直接生成发射目标；
- 在一个函数中维护所有 PID；
- 负责解释视觉和键鼠语义。

## 7. 迁移顺序

### 第一步：稳定数据类型

新增或整理：

```text
OperatorInput.hpp
GimbalInput.hpp
LaunchInput.hpp
GimbalTypes.hpp
LaunchTypes.hpp
```

建议把当前 `GimbalTypes.hpp` 中的 `ControlTask` 重命名为：

```text
GimbalTarget
```

并新增：

```text
LaunchTarget
```

### 第二步：建立 RobotContext

移动以下共享对象：

- `gimbal_fsm`
- `launch_fsm`
- `gimbal_target`
- `launch_target`
- `gimbal_output`
- `launch_output`

暂时不移动 PID 和控制算法对象。

### 第三步：补充 GimbalPlanner

先迁移 `SetTarget_Gimbal()`，保证：

- STOP 保持当前位置；
- MANUAL 目标不变；
- VISION 目标不变；
- DOWN/TRANSFORM 的 Yaw/Pitch 顺序不变；
- Yaw 未到位时 Pitch 保持当前位置。

### 第四步：补充 LaunchPlanner

迁移 `SetTarget_Launch()`：

- STOP；
- CEASEFIRE；
- ONLY；
- AUTO；
- JAM；
- 鼠标左键和拨轮的单发边沿。

### 第五步：拆出 GimbalController

迁移：

- `gimbal_stop()`
- `gimbal_manual()`
- `gimbal_vision()`
- `gimbal_transform()`

先保证输出结果与原代码一致，再考虑控制算法优化。

### 第六步：拆出 LaunchController

迁移：

- `launch_velocity()`
- `launch_angle()`
- `launch_jam()`

并明确拨盘和摩擦轮的输出策略。

### 第七步：重写 ControlTask

最终只保留：

- 输入快照；
- 事件分发；
- Planner 调用；
- Controller 调用；
- 周期调度。

### 第八步：删除旧代码

最后删除：

- `alphabet`；
- `is_change`；
- `is_vision`；
- 旧 `StateUpdate()`；
- 旧的直接 DT7 读取；
- 重复的目标计算；
- 过渡兼容别名。

## 8. 需要特别注意的问题

### 8.1 状态机转换完成不能等同于状态请求

进入 `DOWN` 不代表折叠动作已经完成。

应区分：

```text
DownRequested
    ↓
开始折叠动作
    ↓
Yaw/Pitch 到位
    ↓
CompleteTransition()
    ↓
进入 DOWN 稳定状态
```

### 8.2 STOP 必须能打断所有动作

以下情况都应立即停止：

- 设备离线；
- 底盘/云台进入停止挡；
- 键鼠模式 B 未使能；
- 传感器数据失效；
- 电机出现严重错误。

### 8.3 JAM 不能无限重入

建议增加：

- JAM 触发锁存；
- JAM 冷却时间；
- 卡弹信号恢复确认；
- 恢复前状态保存。

否则卡弹信号一直存在时，可能出现：

```text
JAM -> ONLY/AUTO -> JAM -> ONLY/AUTO
```

反复震荡。

### 8.4 共享数据的并发访问

`SerialTask` 和 `ControlTask` 可能访问同一个 `OperatorInput`。

后续建议：

- 使用完整快照复制；
- 不在控制过程中修改输入；
- 必要时使用临界区或双缓冲；
- 让 Planner 只读取本周期快照。

### 8.5 当前代码版本差异

当前工程中存在不同阶段的控制代码版本：

- 一版使用单 Pitch 目标；
- 一版使用 Pitch1/Pitch2 和 VMC；
- 一版使用 `Motor3508`；
- 一版包含拨盘电机启停和清错流程。

正式迁移时应以当前工程实际使用的 `ControlTask.cpp`、`MotorTask.cpp` 和输出结构为准，不要直接混合其他分支的实现。

## 9. 验收标准

### 云台

- [ ] 所有挡位映射与事件检测一致；
- [ ] `STOP` 不能直接进入 `VISION`；
- [ ] `DOWN` 折叠过程中 Yaw 先动作；
- [ ] Yaw 未到位时 Pitch 不动作；
- [ ] `DOWN -> MANUAL` 先回正 Pitch；
- [ ] 状态转换完成后才调用 `CompleteTransition()`；
- [ ] 视觉和手动目标没有互相覆盖。

### 发射机构

- [ ] 左上右上只进入 `LAUNCH_AUTO`；
- [ ] `ONLY` 不因打出一发自动进入 `CEASEFIRE`；
- [ ] 鼠标左键旧的 ONLY/AUTO 逻辑保留；
- [ ] ONLY 超时后自动进入 AUTO；
- [ ] B 未使能时进入 STOP；
- [ ] STOP 优先于 JAM；
- [ ] JAM 退弹后恢复 ONLY/AUTO；
- [ ] 卡弹不会无限重复触发。

### 架构

- [ ] ControlTask 不再直接读取 `alphabet`；
- [ ] Planner 不直接控制电机；
- [ ] Controller 不负责挡位解释；
- [ ] MotorTask 只发送最终输出；
- [ ] RobotContext 不保存不必要的 PID 实现；
- [ ] 云台和发射机构可以独立测试。

