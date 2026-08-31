#include "LaunchPlanner.hpp"

namespace
{
// 摩擦轮目标转速。两个摩擦轮方向相反，用于夹持并加速弹丸。
constexpr float WHEEL_SPEED = 6000.0f;

// 卡弹退弹时拨盘的目标输出，方向由实际电机控制接口解释。
constexpr float JAM_DIAL_OUTPUT = -20.0f;

// 拨盘转动一格对应的目标角度：9 个弹丸槽位，每槽经过 36 倍减速机构。
constexpr float SLOT_ANGLE = 40.0f * 36.0f;

/*
 * 将发射频率转换为本次控制周期内拨盘应增加的角度。
 *
 * fire_rate 的单位是 Hz，360 / 10 表示每个弹丸槽位占据的圆周角度，
 * 1 是拨盘减速比，0.001f 是发射控制周期。该函数只负责单位换算，
 * 不判断当前是否允许发射。
 */
float HzToAngle(float fire_rate)
{
    return fire_rate * (360.0f / 10.0f) * 1.0f * 0.001f;
}
}

void InitLaunchPlanner()
{
    /*
     * 当前发射规划器使用编译期常量和函数内静态变量，暂时不需要额外初始化。
     * 保留该入口是为了后续集中加载发射机构参数或复位规划器状态。
     */
}

LaunchTarget PlanLaunchTarget(Enum_Launch_States state,
                              const LaunchInput& input,
                              const LaunchTarget& current_target)
{
    // 用于判断本周期是否刚刚进入某个状态。
    static Enum_Launch_States last_state = LAUNCH_STOP;

    // 用于检测鼠标左键释放沿，避免单发触发条件一直保持为真时重复加角度。
    static bool last_mouse_left = false;

    // 以调用者提供的上一周期目标为基础，只修改当前状态负责的输出量。
    LaunchTarget target = current_target;
    const bool entered_state = state != last_state;

    switch (state)
    {
    case LAUNCH_STOP:
        // STOP：拨盘和摩擦轮全部停止。
        target.target_dial = 0.0f;
        target.target_surgewheel[0] = 0.0f;
        target.target_surgewheel[1] = 0.0f;
        break;

    case LAUNCH_CEASEFIRE:
        // CEASEFIRE：禁止拨盘送弹，但保持摩擦轮转动以便快速恢复发射。
        target.target_dial = 0.0f;
        target.target_surgewheel[0] = WHEEL_SPEED;
        target.target_surgewheel[1] = -WHEEL_SPEED;
        break;

    case LAUNCH_JAM:
        // JAM：卡弹处理状态。摩擦轮保持夹弹，拨盘输出退弹力矩/速度。
        target.target_dial = JAM_DIAL_OUTPUT;
        target.target_surgewheel[0] = WHEEL_SPEED;
        target.target_surgewheel[1] = -WHEEL_SPEED;
        break;

    case LAUNCH_ONLY:
        /*
         * ONLY：单发模式。
         * 进入该状态时从当前反馈角度开始规划，避免状态切换瞬间沿用旧目标
         * 导致拨盘突然跳转。
         */
        if (entered_state)
        {
            target.target_dial = input.current_dial_angle;
        }

        /*
         * 单发触发后拨盘前进一个槽位。触发来源包括：
         * 1. 鼠标左键按下；
         * 2. 外部 fire_single 请求；
         * 3. 鼠标左键释放沿，兼容旧逻辑中的单发触发方式。
         *
         * fire_rate 大于 0 才认为发射机构参数有效，避免无效频率下修改目标角度。
         */
        if ((input.mouse_left_pressed || input.fire_single ||
             (last_mouse_left && !input.mouse_left)) && input.fire_rate > 0.0f)
        {
            target.target_dial -= SLOT_ANGLE;
        }

        // 单发时摩擦轮持续运行，拨盘只在上面的触发条件满足时移动一格。
        target.target_surgewheel[0] = WHEEL_SPEED;
        target.target_surgewheel[1] = -WHEEL_SPEED;
        break;

    case LAUNCH_AUTO:
        /*
         * AUTO：连发模式。
         * 首次进入时以当前拨盘角度为基准，之后每个控制周期按照 fire_rate
         * 计算拨盘增量，实现连续送弹。
         */
        if (entered_state)
        {
            target.target_dial = input.current_dial_angle;
        }

        // 连发时两个摩擦轮始终保持相反方向的目标转速。
        target.target_surgewheel[0] = WHEEL_SPEED;
        target.target_surgewheel[1] = -WHEEL_SPEED;

        // 鼠标左键按住时按 fire_rate 连续发射，否则按滚轮输入调节发射速度。
        target.target_dial -= input.mouse_left
            ? HzToAngle(input.fire_rate)
            : input.scroll * HzToAngle(input.fire_rate);
        break;

    default:
        // 未知状态按最安全策略处理：所有执行机构停止。
        target.target_dial = 0.0f;
        target.target_surgewheel[0] = 0.0f;
        target.target_surgewheel[1] = 0.0f;
        break;
    }

    // 保存本周期信息，供下一周期判断状态进入沿和鼠标左键释放沿。
    last_mouse_left = input.mouse_left;
    last_state = state;
    return target;
}
