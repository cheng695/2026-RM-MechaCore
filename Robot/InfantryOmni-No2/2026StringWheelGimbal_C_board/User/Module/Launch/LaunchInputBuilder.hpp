#pragma once

#include "../OperatorInput.hpp"
#include "LaunchInput.hpp"

/* 从公共操作员输入中提取发射机构专用输入。 */
LaunchInput BuildLaunchInput(const OperatorInput& operator_input,
                             bool equipment_online,
                             bool jam_detected);
