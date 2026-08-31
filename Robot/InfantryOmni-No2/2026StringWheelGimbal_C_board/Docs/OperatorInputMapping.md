# 底盘、云台与发射机构键鼠键位说明

本文档记录当前代码中的键鼠和遥控器逻辑。底盘与云台属于两个独立控制板，因此同一个物理按键可以在两块板上具有不同含义。

## 1. 遥控器挡位

| 代码 | 数值 | 含义 |
|---|---:|---|
| `UP` | `1` | 上挡 |
| `MIDDLE` | `3` | 中挡 |
| `DOWN` | `2` | 下挡 |

## 2. 底盘

### 2.1 遥控器挡位

| 左开关 | 右开关 | 底盘状态 |
|---|---|---|
| 左上 `1` | 任意 | `NOTFOLLOW`，不跟随 |
| 左中 `3` | 右上 `1` | `FOLLOW`，跟随 |
| 左中 `3` | 右中 `3` | 由 `Q` 的锁存状态决定 `FOLLOW/NOTFOLLOW` |
| 左中 `3` | 右下 `2` | `NOTFOLLOW` |
| 左下 `2` | 右上 `1` | `FOLLOW` |
| 左下 `2` | 右中 `3` | `NOTFOLLOW` |
| 左下 `2` | 右下 `2` | `STOP` |

设备离线时，底盘无条件请求 `STOP`。

### 2.2 键盘和鼠标

键盘模式条件：

```text
左开关 = 3 且 右开关 = 3
```

| 按键 | 功能 |
|---|---|
| `W` | 前进 |
| `S` | 后退 |
| `A` | 左移 |
| `D` | 右移 |
| `X` | 小陀螺/自旋 |
| `Q` | 切换跟随/不跟随，结果会锁存 |
| `Shift` | 加速，并退出主动补电 |
| `Ctrl` | 切换主动补电 |
| `Ctrl + Shift + F` | 强制裁判系统不可信 |
| `Ctrl + Shift + R` | 强制裁判系统可信 |
| 鼠标左键 | 退出主动补电 |
| 鼠标右键 | 退出主动补电 |

键盘运动输入为：

```text
move_x = W - S
move_y = D - A
rotate = X ? 1.0f : 0.0f
```

非键盘模式下，底盘使用遥控器左摇杆控制平移，使用拨轮控制旋转。

底盘当前没有使用：

```text
E、G、Z、C、V、B
```

## 3. 云台

### 3.1 遥控器挡位

| 左开关 | 右开关 | 云台行为 |
|---|---|---|
| 左上 `1` | 右上 `1` | 请求 `DOWN`，执行折叠 |
| 左上 `1` | 右中 `3` | 视觉有效时进入 `VISION`，否则 `MANUAL` |
| 左上 `1` | 右下 `2` | `MANUAL` |
| 左中 `3` | 右上 `1` | `MANUAL` |
| 左中 `3` | 右中 `3` | 键鼠模式，`V` 控制折叠，视觉可进入 `VISION` |
| 左中 `3` | 右下 `2` | `MANUAL` |
| 左下 `2` | 右上 `1` | `MANUAL` |
| 左下 `2` | 右中 `3` | `MANUAL` |
| 左下 `2` | 右下 `2` | `STOP` |

云台状态：

| 状态 | 含义 |
|---|---|
| `STOP` | 云台停止或失能 |
| `DOWN` | 云台折叠状态 |
| `MANUAL` | 手动控制 |
| `VISION` | 视觉控制 |

`STOP` 和 `DOWN` 不能直接进入 `VISION`。从 `STOP` 或 `DOWN` 回到 `MANUAL` 时，需要由控制层完成 Pitch 回正流程。

### 3.2 `V` 键折叠逻辑

`V` 只在键鼠模式下响应，并使用上升沿检测：

```text
初始：UP，对应 MANUAL
第一次按下 V：DOWN
第二次按下 V：UP，对应 MANUAL
```

离开键鼠模式时，`down_state` 不会自动清零；重新进入键鼠模式后继续使用上一次的折叠状态。

### 3.3 视觉进入条件

```cpp
vision_enabled = vision.getVisionFlag() &&
                  (!keyboard_mode || mouse_right);
```

因此：

- 非键鼠模式：视觉标志有效即可进入视觉模式；
- 键鼠模式：视觉标志有效且鼠标右键按下才允许视觉模式；
- 键鼠模式下如果 `transform_requested` 为真，`DOWN` 优先于 `VISION`。

## 4. 发射机构

### 4.1 遥控器挡位

| 左开关 | 右开关 | 发射状态 |
|---|---|---|
| 左上 `1` | 右上 `1` | 强制 `LAUNCH_AUTO`，只允许手动连发 |
| 左上 `1` | 右中 `3` | 视觉控制停火、单发、连发 |
| 左上 `1` | 右下 `2` | `LAUNCH_STOP` |
| 左中 `3` | 右上 `1` | `LAUNCH_AUTO`，遥控器连发 |
| 左中 `3` | 右中 `3` | 键鼠模式，必须先通过 `B` 使能 |
| 左中 `3` | 右下 `2` | `LAUNCH_ONLY`，遥控器单发 |
| 左下 `2` | 任意 | `LAUNCH_STOP` |

发射状态：

| 状态 | 作用 |
|---|---|
| `LAUNCH_STOP` | 发射机构停止 |
| `LAUNCH_CEASEFIRE` | 摩擦轮预旋，拨盘不送弹 |
| `LAUNCH_ONLY` | 单发模式 |
| `LAUNCH_AUTO` | 连发模式 |
| `LAUNCH_JAM` | 卡弹退弹 |

### 4.2 `B` 键总使能

`B` 在键鼠模式下使用上升沿翻转：

```text
初始：launch_enabled = false
第一次按下 B：launch_enabled = true，进入 CEASEFIRE
第二次按下 B：launch_enabled = false，进入 STOP
```

没有先按 `B` 时，键鼠模式不会发射。

### 4.3 键鼠模式下的发射

在左中右中且 `B` 已使能时：

| 条件 | 结果 |
|---|---|
| 视觉有效 | 使用视觉端的停火/单发/连发指令 |
| 非视觉，鼠标左键按下 | 请求 `LAUNCH_AUTO` |
| 非视觉，按住 `Z` | 请求 `LAUNCH_ONLY` |
| 非视觉，按住 `X` | 请求 `LAUNCH_AUTO` |
| 无上述输入 | `LAUNCH_CEASEFIRE` |

鼠标左键和 `Z` 同时产生请求时，鼠标左键连发优先。

### 4.4 视觉发射

视觉有效时：

| 视觉模式 | 发射状态 |
|---:|---|
| `0` | `LAUNCH_CEASEFIRE` |
| `1` | `LAUNCH_AUTO` |
| `2` | `LAUNCH_ONLY` |

### 4.5 鼠标左键的 ONLY/AUTO 逻辑

鼠标左键上升沿触发时：

```text
如果上一个状态是 ONLY，且此前检测到鼠标左键释放过：
    保持 ONLY
否则：
    进入 AUTO
```

鼠标左键的按下沿和释放信息由 `SerialTask` 记录，再由 `LaunchInputBuilder` 传给发射事件检测器。

### 4.6 ONLY 超时转 AUTO

`LAUNCH_ONLY` 持续超过 `4000` 个发射控制周期后，自动转为 `LAUNCH_AUTO`：

```text
4000 × 5 ms = 约 20 s
```

这个逻辑与是否已经检测到一发弹丸无关，不会因为单发完成而回到 `LAUNCH_CEASEFIRE`。

### 4.7 JAM 卡弹处理

事件优先级：

```text
STOP > JAM > 普通发射模式
```

处理流程：

1. 设备离线或进入停止挡位，立即进入 `LAUNCH_STOP`；
2. 在 `ONLY` 或 `AUTO` 中检测到卡弹，进入 `LAUNCH_JAM`；
3. `JAM` 反转约 150 ms；
4. 完成后恢复进入卡弹前的 `ONLY` 或 `AUTO`；
5. `JAM` 期间普通模式请求不会打断退弹，`STOP` 可以打断退弹。

## 5. 输入分层

当前输入链路为：

```text
OperatorInput
    ├── BuildGimbalInput() -> GimbalInput
    └── BuildLaunchInput() -> LaunchInput
```

职责划分：

| 模块 | 职责 |
|---|---|
| `SerialTask` | 读取遥控器、键盘、鼠标和视觉数据 |
| `GimbalInputBuilder` | 提取云台相关输入 |
| `LaunchInputBuilder` | 提取发射相关输入 |
| `GimbalEventDetector` | 生成云台事件 |
| `LaunchEventDetector` | 生成发射事件 |
| `GimbalStateMachine` | 管理云台状态和折叠转换 |
| `LaunchStateMachine` | 管理发射状态和卡弹恢复 |
| 控制/规划层 | 根据状态产生电机目标和单发拨盘动作 |

## 6. 当前迁移注意事项

云台旧控制层中仍存在部分历史代码，尚未完全迁移到新的输入和事件接口：

- 旧的 `alphabet` 数组引用；
- 旧的云台目标字段；
- 部分单发拨盘逻辑仍直接读取 `DT7`；
- 发射和云台执行层尚未完全从 `ControlTask.cpp` 拆出。

因此本文档描述的是当前新输入/事件结构及已经确认的键位规则，完成控制层迁移后应再进行一次实际代码与文档对照检查。

