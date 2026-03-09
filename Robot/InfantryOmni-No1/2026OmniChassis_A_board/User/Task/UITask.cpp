#include "UITask.hpp"

// 引入系统状态变量（根据你的实际控制框架可能需要 extern 或 include）
// extern uint8_t shoot_mode; 
// extern float gimbal_pitch;

using namespace RM_RefereeSystem;

// 画一个精美的准星 (静态UI，只需画一次，或者为了防丢包定期发)
void DrawStaticUI()
{
    // 不在这里 DeleteAll，防止定期发包时屏幕狂闪
    // RM_RefereeSystemDelete(DeleteAll, 0);
    // osDelay(50); // 裁判系统处理需要时间

    RM_RefereeSystemSetColor(ColorGreen);
    RM_RefereeSystemSetWidth(2);
    RM_RefereeSystemSetOperateTpye(OperateAdd);

    // 准备 5 个图形的容器（准星：1个中心点 + 4根短线）
    // 屏幕中心在 (960, 540)
    graphic_data_struct_t crosshair[5];
    crosshair[0] = RM_RefereeSystemSetCircle((char*)"C01", 0, 960, 540, 4); // 中心准星圆点
    // 十字准线 (留出中间一段空隙)
    crosshair[1] = RM_RefereeSystemSetLine((char*)"L01", 0, 960, 550, 960, 600); // 上
    crosshair[2] = RM_RefereeSystemSetLine((char*)"L02", 0, 960, 530, 960, 480); // 下
    crosshair[3] = RM_RefereeSystemSetLine((char*)"L03", 0, 940, 540, 890, 540); // 左
    crosshair[4] = RM_RefereeSystemSetLine((char*)"L04", 0, 980, 540, 1030, 540);// 右

    // 发送画好的静态UI包
    RM_RefereeSystemSendDataN(crosshair, 5);
    osDelay(50);

    // 首先要Add这个文字，否则后面没法Revise
    RM_RefereeSystemSetColor(ColorYellow);
    RM_RefereeSystemSetStringSize(20);
    RM_RefereeSystemSetOperateTpye(OperateAdd);
    ext_client_custom_character_t mode_str = RM_RefereeSystemSetStr((char*)"S01", 1, (char*)"MODE: SPIN", 100, 900);
    RM_RefereeSystemSendStr(mode_str);
}

// 动态刷新文字状态 (定时刷新)
void UpdateDynamicUI()
{
    RM_RefereeSystemSetColor(ColorYellow);
    RM_RefereeSystemSetStringSize(20);
    
    // 修改现有文字，而不是一直Add
    RM_RefereeSystemSetOperateTpye(OperateRevise);

    // 发送更新字符串
    ext_client_custom_character_t mode_str = RM_RefereeSystemSetStr((char*)"S01", 1, (char*)"MODE: SPIN", 100, 900);
    RM_RefereeSystemSendStr(mode_str);

}


extern "C" {
void UI(void const * argument)
{
    // ===== 最小化诊断测试 =====
    // 不等待 robot_id，直接 delay 3秒让裁判系统稳定
    osDelay(3000);

    for(;;)
    {
        // 每2秒发送一条简单的绿色线段，用来测试 TX 通道是否通畅
//        RM_RefereeSystemSetColor(ColorGreen);
//        RM_RefereeSystemSetWidth(3);
//        RM_RefereeSystemSetOperateTpye(OperateAdd);

//        graphic_data_struct_t test_line = RM_RefereeSystemSetLine((char*)"T01", 0, 800, 400, 1100, 700);
//        RM_RefereeSystemSendDataN(&test_line, 1);

        osDelay(2000);
    }
}
}


// void RefeeTask(void *argument)
// {
//     UI::Static::UI_static.Init();
//     bool last_ref_ready = false;
//     bool last_f5 = false;
//     uint32_t last_periodic_rebuild_ms = HAL_GetTick();

//     for (;;)
//     {
//         const uint32_t now_ms = HAL_GetTick();
//         const bool ref_ready = UI::UI_send_queue.referee_id_ready();
//         const bool f5_now = Gimbal_to_Chassis_Data.getF5();

//         bool request_hard_rebuild = false;
//         bool request_soft_rebuild = false;
//         // 裁判链路由未就绪变为就绪时，触发一次硬重建。
//         if (ref_ready && !last_ref_ready) {
//             request_hard_rebuild = true;
//         }
//         // 全局 F5 上升沿触发硬重建（不依赖底盘模式）。
//         if (f5_now && !last_f5) {
//             request_hard_rebuild = true;
//         }
//         // 周期性自愈：客户端重启会清空 UI，目标端没有显式回调。
//         if (ref_ready && (now_ms - last_periodic_rebuild_ms >= 5000U)) {
//             request_soft_rebuild = true;
//             last_periodic_rebuild_ms = now_ms;
//         }

//         if (request_hard_rebuild &&
//             !UI::UI_send_queue.is_Delete_all &&
//             UI::UI_send_queue.size == 0 &&
//             UI::UI_send_queue.wz_size == 0) {
//             UI::UI_send_queue.setDeleteAll();
//             UI::Static::UI_static.Init();
//         } else if (request_soft_rebuild &&
//                    !UI::UI_send_queue.is_Delete_all &&
//                    UI::UI_send_queue.size == 0 &&
//                    UI::UI_send_queue.wz_size == 0) {
//             // 软重建：仅重发静态 Add，不强制清屏。
//             UI::Static::UI_static.Init();
//         }

//         last_ref_ready = ref_ready;
//         last_f5 = f5_now;

//         UI::Dynamic::UI_dynamic.darw_UI();
//         UI::UI_send_queue.send_wz();
//         UI::UI_send_queue.send();

//         // 静态图元和文本发送完毕后，解锁动态层更新。
//         if (!UI::UI_send_queue.is_up_ui &&
//             UI::UI_send_queue.size == 0 &&
//             UI::UI_send_queue.wz_size == 0) {
//             UI::UI_send_queue.is_up_ui = true;
//         }

//         osDelay(10);
//     }
// }
