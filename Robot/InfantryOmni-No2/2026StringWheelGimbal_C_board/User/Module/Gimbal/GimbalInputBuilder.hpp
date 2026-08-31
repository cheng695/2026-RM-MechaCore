#pragma once

#include "../OperatorInput.hpp"
#include "GimbalInput.hpp"

/* 从公共操作员输入中提取云台专用输入。 */
GimbalInput BuildGimbalInput(const OperatorInput& operator_input);
