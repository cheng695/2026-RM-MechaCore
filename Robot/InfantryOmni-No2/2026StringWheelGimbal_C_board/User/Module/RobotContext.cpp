#include "RobotContext.hpp"
#include "Gimbal/GimbalTypes.hpp"
#include "core/APP/Heat_Detector/Heat_Control_Private.hpp"

/* 鏈哄櫒浜虹骇鍏变韩瀵硅薄闆嗕腑瀹氫箟锛岄伩鍏嶅涓换鍔″垎鍒淮鎶ゅ悓涓€浠界姸鎬併€?*/
Gimbal_FSM gimbal_fsm;
Launch_FSM launch_fsm;
GimbalTarget gimbal_planner_target;
LaunchTarget launch_planner_target;

Output_gimbal gimbal_output;
Output_launch launch_output;

Mit_K gimbal_k;
APP::Heat_Control_Private heat_control(20, 100, 15.0f, 1.0f, 0.001f);
