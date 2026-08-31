#pragma once

#include "LaunchInput.hpp"
#include "LaunchStateMachine.hpp"

/* 根据当前操作员输入和卡弹检测结果生成发射机构事件。 */
LaunchEvent ResolveLaunchEvent(const LaunchInput& input,
                               Enum_Launch_States current_state);

/* 仅在目标事件发生变化时返回事件，避免重复分发相同模式请求。 */
LaunchEvent DetectLaunchEvent(const LaunchInput& input,
                              Enum_Launch_States current_state);
