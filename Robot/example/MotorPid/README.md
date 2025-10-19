# MotorPid 示例工程

## 项目概述

本示例工程演示了如何使用 PID 和 ADRC（自抗扰控制器）两种控制算法对 GM6020 电机进行位置和速度双环控制。通过状态机模式，可以在 DISABLE、PID 和 ADRC 三种控制模式之间切换。

## 核心文件

- **motorPidTask.cpp** - 电机控制任务实现
- **motorPidTask.hpp** - 电机控制任务头文件
- **config.h** - 配置文件，定义电机实例和硬件接口

## 代码架构

### 1. 状态机模式

项目采用有限状态机（FSM）模式，定义了三种运行状态：

```cpp
enum MotorPid_Status
{
    DISABLE,  // 禁用状态：电机跟随当前位置和速度
    PID,      // PID控制模式：使用传统 PID 进行双环控制
    ADRC,     // ADRC控制模式：位置环使用 PID，速度环使用 ADRC
};
```

### 2. 类结构

`MotorPid` 类继承自 `Class_FSM`，包含以下主要成员：

#### 控制器实例
- `pid_pos` - 位置环 PID 控制器
- `pid_vel` - 速度环 PID 控制器
- `adrc_vel` - 速度环 ADRC 控制器

#### 控制目标
- `target_pos` - 目标位置（角度，单位：度）
- `target_vel` - 目标速度（角速度，单位：rad/s）

#### 核心方法
- `UpData()` - 主更新函数，状态机调度
- `Disable()` - 禁用模式处理
- `PidUpData()` - PID 控制模式更新
- `AdrcUpData()` - ADRC 控制模式更新
- `sendCan()` - CAN 总线数据发送

## 代码流程详解

### 初始化流程

```cpp
MotorPid::MotorPid()
    : pid_pos(0.0, 0.0, 0.0, 0.0),     // 位置环 PID 初始化
      pid_vel(0.0, 0.0, 0.0, 0.0),     // 速度环 PID 初始化
      adrc_vel(ALG::LADRC::TDquadratic(100, 0.001), 0.0, 0.0, 0.0, 0.001f, 0.0)
{
    // 获取 CAN 设备实例
    auto &chassis_can = CAN_INSTANCE.get_device(CHASSIS_CAN);
    // 注册电机的 Parse 函数作为 CAN 接收回调
    Motor6020.registerCallback(&chassis_can);
}
```

**初始化步骤：**
1. 初始化位置环和速度环控制器（参数暂时为 0，需要后续调参）
2. 初始化 ADRC 控制器，配置 TD（跟踪微分器）参数
3. 获取 CAN1 总线实例
4. 注册电机的接收回调函数，使电机能够接收 CAN 反馈数据

### 主任务循环

```cpp
void MotorPidTask(void *argument)
{
    for (;;)
    {
        TASK::MotorPid::motorPid.UpData();  // 调用更新函数
        osDelay(1);                          // 1ms 延时，1kHz 控制频率
    }
}
```

FreeRTOS 任务以 1kHz 频率循环调用 `UpData()` 函数。

### 状态机更新流程

```cpp
void MotorPid::UpData()
{
    Status[Now_Status_Serial].Count_Time++;  // 当前状态计时

    switch (Now_Status_Serial)
    {
    case (MotorPid_Status::DISABLE): {
        Disable();
        break;
    }
    case (MotorPid_Status::PID): {
        PidUpData();
        break;
    }
    case (MotorPid_Status::ADRC): {
        AdrcUpData();
        break;
    }
    }

    sendCan();  // 发送控制指令到电机
}
```

**更新流程：**
1. 状态计时器递增
2. 根据当前状态执行相应的控制逻辑
3. 通过 CAN 总线发送控制指令

### 三种控制模式详解

#### 模式 1: DISABLE（禁用模式）

```cpp
void MotorPid::Disable(void)
{
    target_pos = Motor6020.getAngleDeg(2);      // 读取当前位置
    target_vel = Motor6020.getVelocityRads(2);  // 读取当前速度

    pid_pos.setTarget(target_pos);              // 设置位置目标为当前位置
    pid_vel.setTarget(target_vel);              // 设置速度目标为当前速度

    pid_pos.UpData(target_pos);                 // PID 更新
    pid_vel.UpData(pid_pos.getOutput());        // PID 更新
}
```

**工作原理：**
- 目标值始终跟随电机当前状态
- 控制器输出为 0，电机不受控
- 适用于初始化或紧急停止场景
- 保持控制器状态同步，便于切换到其他模式

#### 模式 2: PID（传统 PID 双环控制）

```cpp
void MotorPid::PidUpData()
{
    float cur_pos = Motor6020.getAngleDeg(2);      // 读取当前位置
    float cur_vel = Motor6020.getVelocityRads(2);  // 读取当前速度

    pid_pos.setTarget(target_pos);                 // 设置位置目标
    pid_vel.setTarget(pid_pos.getOutput());        // 速度目标 = 位置环输出

    pid_pos.UpData(cur_pos);                       // 位置环更新
    pid_vel.UpData(cur_vel);                       // 速度环更新
}
```

**控制结构（串级 PID）：**
```
目标位置 → [位置PID] → 目标速度 → [速度PID] → 电机控制量
              ↑                      ↑
           实际位置                实际速度
```

**工作流程：**
1. 位置环：计算目标位置与实际位置的误差，输出期望速度
2. 速度环：计算期望速度与实际速度的误差，输出电机控制电流

#### 模式 3: ADRC（位置 PID + 速度 ADRC）

```cpp
void MotorPid::AdrcUpData()
{
    float cur_pos = Motor6020.getAngleDeg(2);      // 读取当前位置
    float cur_vel = Motor6020.getVelocityRads(2);  // 读取当前速度

    pid_pos.setTarget(target_pos);                 // 设置位置目标
    adrc_vel.setTarget(pid_pos.getOutput());       // ADRC 目标 = 位置环输出

    pid_pos.UpData(cur_pos);                       // 位置环更新
    adrc_vel.UpData(cur_vel);                      // ADRC 速度环更新
}
```

**控制结构：**
```
目标位置 → [位置PID] → 目标速度 → [ADRC] → 电机控制量
              ↑                      ↑
           实际位置                实际速度
```

**ADRC 优势：**
- 自适应扰动补偿：自动估计和抵消外部扰动
- 更强的鲁棒性：对参数变化不敏感
- 更好的动态响应：TD（跟踪微分器）可以减少超调

### CAN 通信流程

```cpp
void MotorPid::sendCan(void)
{
    // 获取 CAN 设备实例
    auto &can_motor = CAN_INSTANCE.get_device(CHASSIS_CAN);

    // 发送控制数据到电机
    Motor6020.sendCAN(&can_motor);
}
```

**通信机制：**
- **接收：** 电机反馈数据通过 CAN 中断自动接收，回调函数解析数据
- **发送：** 每个控制周期（1ms）发送一次控制指令
- **电机 ID：** GM6020 配置为 ID 1-4，发送 ID 为 0x1FF

## 控制器参数说明

### PID 参数

```cpp
PID(float kp, float ki, float kd, float max)
```

- `kp` - 比例增益：影响响应速度，越大响应越快，但过大会振荡
- `ki` - 积分增益：消除稳态误差，但过大会导致超调
- `kd` - 微分增益：抑制超调，改善动态性能
- `max` - 输出限幅：防止控制量过大

**典型参数参考：**
- 位置环：`Kp=10.0, Ki=0.0, Kd=0.5, Max=100.0`
- 速度环：`Kp=50.0, Ki=1.0, Kd=0.0, Max=16000.0`

### ADRC 参数

```cpp
Adrc(TDquadratic td, float Kp, float wc, float b0, float h, float max)
```

- `td` - 跟踪微分器：
  - `r` - 跟踪速度因子（100-500），越大跟踪越快
  - `h` - 采样周期（0.001s = 1ms）
- `Kp` - 控制增益：类似 PID 的 Kp
- `wc` - 观测器带宽：ESO 的观测速度，建议 50-200
- `b0` - 控制增益补偿：通常设为 1
- `h` - 采样周期
- `max` - 输出限幅

**典型参数参考：**
- 速度环：`r=100, Kp=50.0, wc=100.0, b0=1.0, h=0.001, max=16000.0`

## 使用指南

### 1. 硬件连接

- STM32F407 开发板
- GM6020 电机（ID 配置为 1-4）
- CAN1 接口连接电机

### 2. 参数调试步骤

**步骤 1：DISABLE 模式测试**
1. 上电后默认进入 DISABLE 模式
2. 观察电机是否能自由转动
3. 检查 CAN 通信是否正常（通过日志或调试器）

**步骤 2：PID 速度环调试**
1. 将位置环参数设为 0
2. 直接给定速度目标
3. 先调 Kp，再调 Ki，最后微调 Kd

**步骤 3：PID 位置环调试**
1. 在速度环稳定的基础上
2. 调整位置环 Kp 和 Kd
3. 观察位置跟随效果

**步骤 4：ADRC 调试**
1. 先设置较小的 TD 参数 r（如 50）
2. 调整 ADRC Kp 参数
3. 逐步增大 wc 观察效果
4. 最后优化 TD 的 r 参数

### 3. 状态切换

通过修改 `Now_Status_Serial` 变量切换状态：
```cpp
motorPid.Now_Status_Serial = MotorPid_Status::PID;   // 切换到 PID 模式
motorPid.Now_Status_Serial = MotorPid_Status::ADRC;  // 切换到 ADRC 模式
```

### 4. 目标设置

```cpp
motorPid.target_pos = 180.0f;  // 设置目标位置为 180 度
```

## 扩展功能

### 1. 添加更多电机

在 `config.h` 中添加电机实例，并在 `MotorPid` 类中添加对应的控制器。

### 2. 实现上位机控制

通过串口或其他通信方式接收上位机指令，动态设置目标值和切换模式。

### 3. 添加安全保护

- 位置限位保护
- 速度限制
- 电流限制
- 超时保护

### 4. 性能监控

- 记录控制误差
- 统计响应时间
- 分析控制品质

## 注意事项

1. **参数初始化：** 当前控制器参数为 0，需要根据实际电机特性调参
2. **单位统一：** 位置使用度（deg），速度使用弧度/秒（rad/s）
3. **控制频率：** 确保任务运行频率稳定在 1kHz
4. **CAN 总线：** 检查 CAN 波特率配置（通常为 1Mbps）
5. **电机编号：** 代码中使用的是 `Motor6020.getAngleDeg(2)`，确认电机 ID 正确

## 调试技巧

1. **使用日志：** 通过 `LOG` 实例输出关键变量
2. **波形观测：** 使用 Ozone 调试器或上位机绘制实时曲线
3. **单步调试：** 验证控制逻辑是否正确执行
4. **参数记录：** 记录每次调参结果，建立参数库

## 常见问题

**Q: 电机抖动？**
- A: 检查 PID 参数是否过大，尝试减小 Kp 和 Ki

**Q: 位置跟随慢？**
- A: 增大位置环 Kp，或提高速度环最大输出

**Q: ADRC 效果不明显？**
- A: 检查 wc 参数是否足够大，TD 的 r 是否合理

**Q: CAN 通信失败？**
- A: 检查硬件连接、波特率配置、终端电阻

## 参考资料

- [PID 控制原理](../../core/Alg/PID/README.md)
- [ADRC 理论基础](../../core/Alg/ADRC/)
- [GM6020 电机手册](../../core/BSP/Motor/Dji/)
- [CAN 通信协议](../../core/HAL/CAN/README.md)

## 版本历史

- **v1.0** - 初始版本，实现基础 PID 和 ADRC 控制
- 待完善：参数自整定、多电机协同控制

---

**作者：** MechaCore Team  
**更新日期：** 2025-10-19  
**许可证：** MIT

