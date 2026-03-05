#include "UITask.hpp"

// 引入系统状态变量（根据你的实际控制框架可能需要 extern 或 include）
// extern uint8_t shoot_mode; 
// extern float gimbal_pitch;

using namespace RM_RefereeSystem;

// 画一个精美的准星 (静态UI，只需画一次)
void DrawStaticUI()
{
    // 清空图层0的所有残留图形
    RM_RefereeSystemDelete(DeleteAll, 0);
    osDelay(50); // 裁判系统处理需要时间

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
}

// 动态刷新文字状态 (定时刷新)
void UpdateDynamicUI()
{
    RM_RefereeSystemSetColor(ColorYellow);
    RM_RefereeSystemSetStringSize(20);
    
    // 修改现有文字，而不是一直Add
    RM_RefereeSystemSetOperateTpye(OperateRevise);

    // 假设这是一个模式显示
    ext_client_custom_character_t mode_str = RM_RefereeSystemSetStr((char*)"S01", 1, (char*)"MODE: SPIN", 100, 900);
    RM_RefereeSystemSendStr(mode_str);

    
}


extern "C" {
void UI(void const * argument)
{
    // 等待裁判系统上电初始化完成
    osDelay(3000); 

    // 先画静态且永远不会变的框架
    DrawStaticUI();

    for(;;)
    {
        // 动态数据建议降频发送 (例如 5Hz, 200ms)
        //UpdateDynamicUI();

        osDelay(100); 
    }
}
}