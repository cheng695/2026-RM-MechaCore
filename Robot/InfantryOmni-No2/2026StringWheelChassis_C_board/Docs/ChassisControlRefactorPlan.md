# 底盘控制重构建议

## 结论

当前底盘代码最需要优先修改三件事：

1. 状态机应改为事件触发，而不是每个控制周期重新根据输入组合推导状态。
2. `ControlTask.cpp` 需要拆分职责，避免把输入解释、状态切换、目标规划、底盘闭环和功率控制都放在同一个文件里。
3. `alphabet[28]` 这种魔法数组应封装为有语义的操作员输入结构体，否则后续维护状态机和功率策略会越来越困难。

另外还应检查板间通信接收回调、命名规范、全局变量暴露范围和重复代码。这些问题不会立刻表现为编译错误，但会让调车、排故和后续扩展非常痛苦。

## 当前问题

### 1. 状态机不是事件驱动

位置：

- `User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.hpp`
- `User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.cpp`
- `User/Task/ControlTask.cpp`

当前 `Chassis_FSM::StateUpdate(uint8_t left, uint8_t right, bool equipment_online, bool *alphabet)` 每 1ms 被调用一次，然后根据当前开关、在线状态和 `alphabet[16]` 直接覆盖 `State_chassis`。

这会带来几个问题：

- 状态机依赖输入细节，`alphabet[16]` 的含义只有调用者和读代码的人知道。
- 状态变化没有触发原因，无法区分是遥控器开关变化、Q 键切换、设备离线还是设备恢复。
- 状态进入动作和退出动作不好写，例如进入 `STOP` 时清 PID、恢复在线时重新初始化斜坡规划。
- `NAVIGATION` 状态已经定义，但 `ControlTask.cpp` 没有对应执行逻辑，最后会走 default 停止。

### 2. `ControlTask.cpp` 职责过多

位置：

- `User/Task/ControlTask.cpp`
- `User/Task/ControlTask.hpp`

当前 `ControlTask.cpp` 同时负责：

- 检查设备在线。
- 初始化和更新状态机。
- 读取 DT7 摇杆、鼠标键盘和 `alphabet`。
- 根据 FOLLOW / NOTFOLLOW / STOP 规划目标速度。
- 做云台角度补偿和坐标旋转。
- 做斜坡规划。
- 做底盘 IK / FK。
- 做轮子 PID。
- 做功率限制、能量环和超电指令。
- 更新最终电机输出。

这些逻辑放在一起后，任何一个小改动都容易碰到其他层。例如只想改键鼠映射，却会进入状态机和功率控制代码；只想改功率策略，却会看到大量运动规划细节。

### 3. FOLLOW 和 NOTFOLLOW 的执行代码重复

位置：

- `chassis_follow()`
- `chassis_notfollow()`

两个函数中 FK、IK、轮速 PID、输出 clamp、功率策略、能量环、超电下发、3508 电流衰减几乎完全相同。FOLLOW / NOTFOLLOW 的主要差异其实只在目标 `target_rotation` 如何生成：

- FOLLOW：没有手动旋转时使用云台跟随 PID。
- NOTFOLLOW：旋转只来自操作输入。

所以应合并底盘执行层，把差异放在目标规划层。

### 4. `alphabet[28]` 可读性和封装性差

位置：

- `User/Task/SerialTask.cpp`
- `User/Task/SerialTask.hpp`
- `User/Task/ControlTask.cpp`
- `User/core/Alg/PowerControl/PowerControl.hpp`

当前索引含义包括：

| 索引 | 当前含义 |
| --- | --- |
| `alphabet[22]` | W，前进 |
| `alphabet[18]` | S，后退 |
| `alphabet[0]` | A，左移 |
| `alphabet[3]` | D，右移 |
| `alphabet[23]` | X，小陀螺 |
| `alphabet[26]` | Shift，加速/超电 |
| `alphabet[27]` | Ctrl 触发的主动补电模式 |
| `alphabet[16]` | Q toggle，底盘是否跟随 |
| `alphabet[5]` | 不信任裁判系统 |
| `alphabet[17]` | 信任裁判系统 |

问题是这些索引没有类型保护，也没有业务语义。状态机、功率策略和目标规划都直接依赖数组索引，会让模块边界变得很模糊。

### 5. 板间通信接收回调疑似未解析数据

位置：

- `User/Task/CommunicationTask.cpp`
- `User/Task/CommunicationTask.hpp`
- `User/Task/Callback.cpp`

`CommunicationTask.cpp` 中 `BoardCommunicationInit()` 注册了 UART6 接收回调，但 lambda 为空：

```cpp
uart6.register_rx_callback([](const HAL::UART::Data &data)
{
});
```

而 `BoardCommunication::board_rx_`、`Cboard.updateTimestamp()`、`DT7.parseData()` 没有在这个回调里被调用。若没有其他地方处理，以下数据可能不会更新：

- `Cboard.GetYawAngle()`
- `Cboard.GetScroll()`
- `Cboard.isConnected()`
- `DT7` 遥控器/键鼠数据

这会影响 FOLLOW 角度、遥控器在线判断、小陀螺禁止逻辑和状态切换。

### 6. 命名和接口不够一致

位置：

- `main_loop_gimbal()` 位于底盘控制文件中，命名应改为 `ChassisControlLoop()`。
- `ControlTask` 既像任务名，又是底盘目标结构体，建议改为 `ChassisTarget`。
- `Output_chassis` 建议改为 `ChassisOutput`。
- `PI_`、`STANDARD` 使用宏，建议改为 `constexpr`。
- `SetTarget()` 名字太泛，建议改为 `PlanChassisTarget()`。

## 推荐目标架构

建议先保持现有算法不大改，只移动边界和接口。推荐结构如下：

```text
SerialTask / CommunicationTask
    |
    v
OperatorInput / BoardInput
    |
    v
ChassisEventDetector
    |
    v
Chassis_FSM
    |
    v
ChassisPlanner
    |
    v
ChassisController
    |
    v
ChassisPowerLimiter
    |
    v
MotorTask
```

各层职责：

| 模块 | 职责 |
| --- | --- |
| `SerialTask` | 只解析遥控器、键鼠和裁判系统数据，生成有语义的输入状态 |
| `CommunicationTask` | 只负责板间通信和 vofa 发送，不直接参与控制决策 |
| `Chassis_FSM` | 只接收事件和上下文，输出当前底盘状态 |
| `ChassisPlanner` | 根据状态和输入生成 `ChassisTarget` |
| `ChassisController` | 根据 `ChassisTarget` 计算 3508 原始输出 |
| `ChassisPowerLimiter` | 根据超电、裁判系统和功率策略限制输出 |
| `MotorTask` | 只发送最终电流到 CAN |

## 修改方法

### 第一步：封装操作员输入

新增建议文件：

- `User/Task/OperatorInput.hpp`

建议定义：

```cpp
#pragma once

#include <stdint.h>

struct OperatorInput
{
    float move_x = 0.0f;
    float move_y = 0.0f;
    float rotate = 0.0f;

    bool keyboard_mode = false;
    bool spin = false;
    bool boost = false;
    bool charge = false;
    bool follow_enabled = true;
    bool force_referee_untrusted = false;
    bool force_referee_trusted = false;

    uint8_t left_switch = 2;
    uint8_t right_switch = 2;
};
```

然后把 `SerialTask.cpp` 中 `KeyProcess(bool *alphabet)` 改成更新一个全局或单例 `OperatorInput operator_input`。

短期兼容方案：

- 先保留 `alphabet[28]`，避免一次性改动太大。
- 新增 `OperatorInput BuildOperatorInput()`，从 DT7 和 alphabet 生成语义结构。
- 后续再逐步删除 `alphabet` 对外暴露。

示例：

```cpp
OperatorInput BuildOperatorInput()
{
    OperatorInput input;
    input.left_switch = DT7.get_s1();
    input.right_switch = DT7.get_s2();
    input.keyboard_mode = (DT7.get_s1() == 3 && DT7.get_s2() == 3);
    input.follow_enabled = alphabet[16];
    input.spin = alphabet[23];
    input.boost = alphabet[26];
    input.charge = alphabet[27];
    input.force_referee_untrusted = alphabet[5];
    input.force_referee_trusted = alphabet[17];

    if (input.keyboard_mode)
    {
        input.move_x = static_cast<float>(alphabet[22]) - static_cast<float>(alphabet[18]);
        input.move_y = static_cast<float>(alphabet[3]) - static_cast<float>(alphabet[0]);
        input.rotate = alphabet[23] ? 1.0f : 0.0f;
    }
    else
    {
        input.move_x = -DT7.get_left_x();
        input.move_y = DT7.get_left_y();
        input.rotate = DT7.get_scroll_();
    }

    return input;
}
```

### 第二步：把状态机改为事件驱动

修改文件：

- `User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.hpp`
- `User/core/BSP/Common/FiniteStateMachine/FiniteStateMachine_chassis.cpp`

建议新增：

```cpp
enum class ChassisEvent
{
    None = 0,
    EquipmentLost,
    EquipmentRecovered,
    SwitchChanged,
    FollowToggleChanged,
};

struct ChassisEventContext
{
    uint8_t left_switch = 2;
    uint8_t right_switch = 2;
    bool equipment_online = false;
    bool follow_enabled = true;
};
```

状态机对外提供：

```cpp
void Dispatch(ChassisEvent event, const ChassisEventContext& context);
```

内部保留一个纯函数用于根据上下文决定目标状态：

```cpp
Enum_Chassis_States ResolveState(const ChassisEventContext& context);
void TransitTo(Enum_Chassis_States next_state);
```

示例实现：

```cpp
void Chassis_FSM::Dispatch(ChassisEvent event, const ChassisEventContext& context)
{
    if (event == ChassisEvent::None)
    {
        return;
    }

    const auto next_state = ResolveState(context);
    TransitTo(next_state);
}
```

事件由控制循环检测：

```cpp
ChassisEvent DetectChassisEvent(const OperatorInput& input, bool equipment_online)
{
    static uint8_t last_left = 2;
    static uint8_t last_right = 2;
    static bool last_online = false;
    static bool last_follow_enabled = true;

    ChassisEvent event = ChassisEvent::None;

    if (!equipment_online && last_online)
    {
        event = ChassisEvent::EquipmentLost;
    }
    else if (equipment_online && !last_online)
    {
        event = ChassisEvent::EquipmentRecovered;
    }
    else if (input.left_switch != last_left || input.right_switch != last_right)
    {
        event = ChassisEvent::SwitchChanged;
    }
    else if (input.follow_enabled != last_follow_enabled)
    {
        event = ChassisEvent::FollowToggleChanged;
    }

    last_left = input.left_switch;
    last_right = input.right_switch;
    last_online = equipment_online;
    last_follow_enabled = input.follow_enabled;

    return event;
}
```

注意：设备离线是安全事件，建议即使状态没变化也允许刷新 STOP 保护逻辑。可以在控制循环里做双保险：

```cpp
if (!equipment_online)
{
    chassis_fsm.ForceStop();
}
```

### 第三步：拆分目标规划

修改文件：

- `User/Task/ControlTask.cpp`
- 可新增 `User/Task/ChassisPlanner.hpp`
- 可新增 `User/Task/ChassisPlanner.cpp`

把当前 `SetTarget()` 拆成：

```cpp
struct ChassisTarget
{
    float translation_x = 0.0f;
    float translation_y = 0.0f;
    float rotation = 0.0f;
};

ChassisTarget PlanChassisTarget(Enum_Chassis_States state, const OperatorInput& input);
```

规划层内部处理：

- STOP 直接输出全 0。
- FOLLOW 中，如果有手动旋转/小陀螺，则 rotation 来自输入。
- FOLLOW 中，如果没有手动旋转，则 rotation 来自 `follow_pid`。
- NOTFOLLOW 中，rotation 始终来自输入。
- 键鼠/遥控器差异只在 `OperatorInput` 中处理，不再散落在 FOLLOW/NOTFOLLOW 分支里。

当前 `CalculateTranslation_xy()`、`CalculateFollow()`、`LimitChassisVector()` 可以先移入 `ChassisPlanner.cpp` 的匿名 namespace 中，避免污染全局符号。

### 第四步：合并 FOLLOW / NOTFOLLOW 执行层

修改文件：

- `User/Task/ControlTask.cpp`

用一个函数替代 `chassis_follow()` 和 `chassis_notfollow()`：

```cpp
void RunChassisController(const ChassisTarget& target)
{
    omni_fk.OmniForKinematics(
        Motor3508.getVelocityRads(1),
        Motor3508.getVelocityRads(2),
        Motor3508.getVelocityRads(3),
        Motor3508.getVelocityRads(4));

    omni_ik.OmniInvKinematics(
        target.translation_x,
        target.translation_y,
        target.rotation,
        0.0f,
        1.0f,
        1.0f);

    for (int i = 0; i < 4; i++)
    {
        wheel_pid[i].UpDate(omni_ik.GetMotor(i), Motor3508.getVelocityRads(i + 1));
        chassis_output.out_wheel[i] = std::clamp(wheel_pid[i].getOutput(), -16384.0f, 16384.0f);
    }
}
```

再把功率控制提取为：

```cpp
void ApplyChassisPowerLimit(const OperatorInput& input);
```

这样 FOLLOW 和 NOTFOLLOW 不再需要两个大函数。

### 第五步：修正板间通信解析

修改文件：

- `User/Task/CommunicationTask.hpp`
- `User/Task/CommunicationTask.cpp`

建议给 `BoardCommunication` 增加：

```cpp
bool Parse(const uint8_t* data, uint16_t size)
{
    if (data == nullptr || size != sizeof(BoardRX))
    {
        return false;
    }

    memcpy(&board_rx_, data, sizeof(BoardRX));
    updateTimestamp();
    return true;
}
```

然后在 `BoardCommunicationInit()` 中：

```cpp
uart6.register_rx_callback([](const HAL::UART::Data &data)
{
    if (Cboard.Parse(data.buffer, data.size))
    {
        DT7.parseData(Cboard.board_rx_.rc_data);
    }
});
```

如果担心 `board_rx_` 暴露过多，可以加 getter：

```cpp
const uint8_t* GetRemoteData() const
{
    return board_rx_.rc_data;
}
```

然后写成：

```cpp
DT7.parseData(Cboard.GetRemoteData());
```

### 第六步：整理功率控制接口

修改文件：

- `User/Task/ControlTask.cpp`
- `User/core/Alg/PowerControl/PowerControl.hpp`

当前功率策略直接吃 `alphabet`，建议逐步改为有语义的结构：

```cpp
struct PowerControlInput
{
    bool supercap_online = false;
    bool referee_online = false;
    bool boost = false;
    bool charge = false;
    bool force_referee_untrusted = false;
    bool force_referee_trusted = false;
    float referee_power_limit = 0.0f;
    float referee_buffer_energy = 0.0f;
    float supercap_energy = 0.0f;
};
```

短期兼容可以先在 `ControlTask.cpp` 中用 `OperatorInput` 重新填充旧 `alphabet` 参数。长期应让 `PowerControlStrategy::Update()` 不再知道 `alphabet`。

### 第七步：规范命名和常量

建议改名：

| 当前名称 | 建议名称 |
| --- | --- |
| `main_loop_gimbal()` | `ChassisControlLoop()` |
| `ControlTask` struct | `ChassisTarget` |
| `Output_chassis` | `ChassisOutput` |
| `SetTarget()` | `PlanChassisTarget()` 或 `UpdateChassisTarget()` |
| `check_online()` | `IsChassisControlReady()` |
| `STANDARD` | `kFollowYawOffset` |
| `PI_` | `kPi` |

建议常量集中：

```cpp
namespace ChassisConfig
{
constexpr float kPi = 3.14159265358979323846f;
constexpr float kFollowYawOffset = -0.22418f;
constexpr float kMaxLinearVelocity = 4.5f;
constexpr float kMaxRotationVelocity = 13.0f;
constexpr float kMotorPhysicalLimit = 5.2f;
constexpr float kWheelRadius = 0.24f;
constexpr float kRemoteRotateDeadzone = 0.05f;
constexpr float kMaxMotorCurrentCommand = 16384.0f;
}
```

## 推荐迁移顺序

### 阶段 1：低风险整理

目标是不改变行为，只提高可读性。

1. 新增 `OperatorInput` 和 `BuildOperatorInput()`。
2. `ControlTask.cpp` 改为使用 `OperatorInput`，但暂时保留 `alphabet`。
3. 合并 `chassis_follow()` / `chassis_notfollow()` 的重复执行代码。
4. 改名 `main_loop_gimbal()` 为 `ChassisControlLoop()`。
5. 把宏改成 `constexpr`。

这一阶段应重点验证：

- STOP 能清零输出。
- FOLLOW 能自动跟随云台角度。
- NOTFOLLOW 能手动旋转。
- 键鼠模式和遥控器模式方向不变。
- Shift、Ctrl、X、Q 的行为不变。

### 阶段 2：事件化状态机

目标是让状态切换可追踪。

1. 在 FSM 中加入 `ChassisEvent` 和 `ChassisEventContext`。
2. 新增 `Dispatch()`、`ResolveState()`、`TransitTo()`。
3. 在控制循环中增加 `DetectChassisEvent()`。
4. 用 `Dispatch()` 替代旧 `StateUpdate()`。
5. 保留旧 `StateUpdate()` 一段时间作为兼容包装，内部转为构造 context 并调用 `Dispatch()`。

兼容包装示例：

```cpp
void Chassis_FSM::StateUpdate(uint8_t left, uint8_t right, bool equipment_online, bool *alphabet)
{
    ChassisEventContext context;
    context.left_switch = left;
    context.right_switch = right;
    context.equipment_online = equipment_online;
    context.follow_enabled = alphabet[16];

    Dispatch(ChassisEvent::SwitchChanged, context);
}
```

等 `ControlTask.cpp` 全部迁移后，再删除旧接口。

### 阶段 3：通信和功率封装

目标是减少隐式全局状态。

1. 给 `BoardCommunication` 增加 `Parse()`。
2. UART6 回调中解析 `BoardRX` 并更新 DT7。
3. 给功率控制增加 `PowerControlInput`。
4. `PowerControlStrategy::Update()` 从 `bool *alphabet` 改为结构体参数。

## 建议的最终控制循环

最终 `Control()` 中的循环可以收敛成：

```cpp
void ChassisControlLoop()
{
    const bool equipment_online = IsChassisControlReady();
    const OperatorInput input = BuildOperatorInput();

    const ChassisEvent event = DetectChassisEvent(input, equipment_online);
    ChassisEventContext context;
    context.left_switch = input.left_switch;
    context.right_switch = input.right_switch;
    context.equipment_online = equipment_online;
    context.follow_enabled = input.follow_enabled;

    chassis_fsm.Dispatch(event, context);

    const ChassisTarget target = PlanChassisTarget(chassis_fsm.Get_Now_State(), input);

    if (chassis_fsm.Get_Now_State() == STOP)
    {
        StopChassis();
        return;
    }

    RunChassisController(target);
    ApplyChassisPowerLimit(input);
}
```

这样 `Control()` 本身只负责调度：

```cpp
extern "C" void Control(void const * argument)
{
    BSP::WATCH_STATE::BuzzerManagerSimple::getInstance().init();
    chassis_fsm.Init();

    for (;;)
    {
        ChassisControlLoop();
        osDelay(1);
    }
}
```

## 额外注意点

### `NAVIGATION` 状态

FSM 中定义了 `NAVIGATION`，但 `ControlTask.cpp` 没有对应 case。建议二选一：

- 如果近期不用导航，先删除或屏蔽 `NAVIGATION`，避免误入 default stop。
- 如果要用，补齐 `PlanNavigationTarget()` 和执行分支。

### 编码问题

当前部分文件注释显示为乱码，说明文件编码和 IDE/终端读取编码不一致。建议统一为 UTF-8，至少保证新增文件使用 UTF-8。否则后续团队协作时注释会越来越不可读。

### 全局变量边界

当前大量对象是全局变量，例如 `chassis_target`、`chassis_output`、`power3508`、`energy_ring`、`DT7`、`Cboard`。嵌入式里全局对象很常见，但建议减少对外暴露：

- 需要跨任务共享的只暴露只读 getter 或明确的输出对象。
- 控制内部使用的 PID、IK、FK、斜坡规划、功率控制对象放在 `.cpp` 的匿名 namespace。
- `chassis_output` 是 MotorTask 的输入，建议后续封装成 `GetChassisOutput()` 或消息/缓冲对象。

## 最小可落地版本

如果只想先做最划算的一版，建议按这个最小集合改：

1. 新增 `OperatorInput`，替代 ControlTask 内部直接读 `alphabet`。
2. 合并 `chassis_follow()` 和 `chassis_notfollow()`。
3. 把 `SetTarget()` 改名并拆成 `PlanChassisTarget()`。
4. FSM 新增 `Dispatch()`，但保留旧 `StateUpdate()` 兼容。
5. 修复 UART6 回调，确保 `Cboard` 和 `DT7` 数据真的被更新。

这五步做完，代码结构会清楚很多，而且不会一下子把所有底层算法都翻掉。

| 左开关 | 右开关 | 底盘 Chassis | 云台 Gimbal | 发射 Launch |
|---|---|---|---|---|
| 左上 `1` | 右上 `1` | `NOTFOLLOW` 不跟随 | 可进入视觉；同时因为 `right == 1`，会进入 `TRANSFORM`，且优先级高于视觉 | 视觉控制发射：视觉给停火/单发/连发 |
| 左上 `1` | 右中 `3` | `NOTFOLLOW` 不跟随 | 可进入视觉 `VISION` | 视觉控制发射：视觉给停火/单发/连发 |
| 左上 `1` | 右下 `2` | `NOTFOLLOW` 不跟随 | 默认手动 `MANUAL` | `LAUNCH_STOP`，发射停止 |
| 左中 `3` | 右上 `1` | 当前代码是 `FOLLOW` 跟随 | 默认手动 `MANUAL` | 遥控器连发 `LAUNCH_AUTO` |
| 左中 `3` | 右中 `3` | 键鼠模式：`Q` 开关决定 `FOLLOW / NOTFOLLOW` | 键鼠模式：`vision_flag` 可进视觉，`transform_flag` 可进变形 | 键鼠模式：需要 `B` 为真，否则发射停止；之后鼠标/视觉/Z/X 控制单发连发 |
| 左中 `3` | 右下 `2` | `NOTFOLLOW` 不跟随 | 默认手动 `MANUAL` | 遥控器单发 `LAUNCH_ONLY` |
| 左下 `2` | 右上 `1` | `FOLLOW` 跟随 | 默认手动 `MANUAL` | `LAUNCH_STOP`，因为发射逻辑里 `left == 2` 直接停 |
| 左下 `2` | 右中 `3` | `NOTFOLLOW` 不跟随 | 默认手动 `MANUAL` | `LAUNCH_STOP` |
| 左下 `2` | 右下 `2` | `STOP` 底盘停止 | `STOP` 云台停止 | `LAUNCH_STOP` |