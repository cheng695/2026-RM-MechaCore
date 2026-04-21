#ifndef UITASK_HPP
#define UITASK_HPP

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "../User/core/APP/Referee/RM_RefereeSystem.h"
#include "../User/core/APP/Referee/RM_RefereeSystemCRC.h"
#include <string.h>

// 超电位置
#define CD_X    960
#define CD_Y    200
#define CD_W    79
#define CD_H    15
#define CD_WZ_X 1000 // 文字
#define CD_WZ_Y 250
// 超电线
#define CDL_X 1260
#define CDL_Y 240
#define CDL_H 50
// 底盘位置
#define CHASSIS_X 760
#define CHASSIS_Y 180
#define CHASSIS_W 50
#define CHASSIS_H 50
// 底盘碰撞位置
#define collide_1       550
#define collide_2       1370
#define collide_magnify 3 // 放大倍率
// 下面的瞄准线
#define aim_x 700
#define aim_y 465

// GY_V小陀螺转速
#define GY_V_X 20
#define GY_V_Y 840
// yd_V小陀螺速速
#define YD_V_X 20
#define YD_V_Y 790
// 摩擦轮on_off
#define MCL_of_X 20
#define MCL_of_Y 740
// 仓门on_off
#define CM_of_X 20
#define CM_of_Y 690
// 拨盘on_off
#define BP_of_X 20
#define BP_of_Y 640
// 超电on_off
#define CD_of_X 20
#define CD_of_Y 340
// 视觉自瞄
#define ZM_of_X 570
#define ZM_of_Y 524

namespace ui_ns
{
    class send_graphic_queue // 发送数据队列
    {
    public:
        TickType_t last_send_time = 0;                                   // 发送时间基准
        uint32_t debug_send_cnt = 0;                                     // 成功发送图形包次数
        uint32_t debug_send_wz_cnt = 0;                                  // 成功发送字符包次数
        RM_RefereeSystem::graphic_data_struct_t graphic_data_struct[50]; // 图层数据
        // 图案
        int8_t size;
        int8_t send_graphic_data_struct_size;
        // 优先把所有文字显示出来
        RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character[20]; // 文字数据
        // 文字
        int8_t wz_size;
        // 刷新图层
        bool is_Delete_all;
        bool is_up_ui = false; // 上锁

    public:
        void add(RM_RefereeSystem::graphic_data_struct_t graphic_data_struct_temp)
        {
            if (size >= 49)
                return;
            memcpy((void *)&graphic_data_struct[size], (void *)&graphic_data_struct_temp,
                   sizeof(RM_RefereeSystem::graphic_data_struct_t));
            size++;
        }
        void add_wz(RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character_temp)
        {
            if (wz_size >= 19)
                return;
            memcpy((void *)&ext_client_custom_character[wz_size], (void *)&ext_client_custom_character_temp,
                   sizeof(RM_RefereeSystem::ext_client_custom_character_t));
            wz_size++;
        }
        bool send()
        {
            if (is_Delete_all == true)
                return false;
            if (wz_size != 0)
                return false;
            if (size == 0)
                return true;
                
            TickType_t now = xTaskGetTickCount();
            if (now - last_send_time < pdMS_TO_TICKS(100))
                return false;
            last_send_time = now;
            
            if (size >= 7)
                send_graphic_data_struct_size = 7;
            else if (size > 2)
                send_graphic_data_struct_size = 5;
            else if (size == 2)
                send_graphic_data_struct_size = 2;
            else if (size == 1)
                send_graphic_data_struct_size = 1;
            else
                send_graphic_data_struct_size = 0;
            if (send_graphic_data_struct_size != 0) {
                RM_RefereeSystem::RM_RefereeSystemSendDataN(graphic_data_struct, send_graphic_data_struct_size);
                size -= send_graphic_data_struct_size;
                debug_send_cnt++; // ++计数器，便于你观察是否真的有在往外发
            }
            if (size < 0)
                size = 0;
            memcpy(graphic_data_struct, (void *)&graphic_data_struct[send_graphic_data_struct_size],
                   sizeof(RM_RefereeSystem::graphic_data_struct_t) * (size));
            memset((void *)&graphic_data_struct[size], 0,
                   sizeof(RM_RefereeSystem::graphic_data_struct_t) * (send_graphic_data_struct_size));
            return true;
        }
        bool send_wz()
        {
            if (is_Delete_all == true)
                return false;
            if (wz_size == 0)
                return true;
                
            TickType_t now = xTaskGetTickCount();
            if (now - last_send_time < pdMS_TO_TICKS(100))
                return false;
            last_send_time = now;
            
            if (wz_size != 0) {
                RM_RefereeSystem::RM_RefereeSystemSendStr(ext_client_custom_character[wz_size - 1]);
                wz_size--;
                debug_send_wz_cnt++; // ++字符发送计数器
            }
            if (wz_size < 0)
                wz_size = 0;
            return true;
        }
        bool send_delet_all()
        {
            if (is_Delete_all == false)
                return true;
                
            TickType_t now = xTaskGetTickCount();
            if (now - last_send_time < pdMS_TO_TICKS(100))
                return false;
            last_send_time = now;
            
            if (is_Delete_all == true) {
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::DeleteAll);
                RM_RefereeSystem::RM_RefereeSystemDelete(RM_RefereeSystem::DeleteAll, 0);
                RM_RefereeSystem::RM_RefereeSystemClsToop();
                is_Delete_all = false;
            }
            return true;
        }

        void setDeleteAll()
        {
            is_Delete_all = true;
        }
    };

    class darw_dynamic
    {
        public:
            private:
                float sin_tick;
                int16_t pitch_out, cap_out, speed_out;
                int16_t yaw_e_rad;
                int16_t yawa;
                int16_t yawb;
                float vel;
    };

    class darw_static : public send_graphic_queue
    {
        private:

        public:
            void PitchLine()
            {
                is_up_ui = false;
                size = 0; // 复位队列
                wz_size = 0;

                // 准备静态UI属性
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
                
                // 瞄准点
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
                add(RM_RefereeSystem::RM_RefereeSystemSetCircle((char*)"W", 0, 954, 495, 6));

                // // pitch初始化 (绘制pitch指示)
                // RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorAmaranth);
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"power", 1, 100, 100 + 2, 960, 540, 380, 380));

                // RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"limPwr", 1, 120, 120 + 2, 960, 540, 380, 380));

                // // 超电初始化
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"cd_Ini", 3, 271, 310, 960, 540, 380, 380));

                // // 小陀螺初始化
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"gy_Ini", 2, 0, 360, 1450, 750, 80, 80));

                // // 转速条初始化
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(35);
                // add(RM_RefereeSystem::RM_RefereeSystemSetLine((char*)"dp1", 0, 1450, 690, 1450, 750 + 61));

                // /***************************绘制静态UI***************************/
                // // pitch刻度
                // RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_130", 0, 50, 51, 960, 540, 380, 380));
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_120", 1, 60, 61, 960, 540, 380, 380));
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_110", 2, 70, 71, 960, 540, 380, 380));
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_100", 3, 80, 81, 960, 540, 380, 380));
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_90", 4, 90, 91, 960, 540, 380, 380));
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_80", 5, 100, 101, 960, 540, 380, 380));
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_60", 6, 110, 111, 960, 540, 380, 380));
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_50", 7, 120, 121, 960, 540, 380, 380));
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"a_40", 8, 130, 131, 960, 540, 380, 380));

                // // pitch刻度数字
                // RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
                // RM_RefereeSystem::RM_RefereeSystemSetStringSize(10);
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
                // add(RM_RefereeSystem::RM_RefereeSystemSetInt((char*)"w_130", 0, 120, 1210, 770));
                // add(RM_RefereeSystem::RM_RefereeSystemSetInt((char*)"w_110", 1, 90, 1270, 660));
                // add(RM_RefereeSystem::RM_RefereeSystemSetInt((char*)"w_90", 2, 60, 1300, 540));
                // add(RM_RefereeSystem::RM_RefereeSystemSetInt((char*)"w_70", 3, 30, 1280, 420));
                // add(RM_RefereeSystem::RM_RefereeSystemSetInt((char*)"w_50", 4, 0, 1210, 310));

                // // 超电上限位
                // RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
                // add(RM_RefereeSystem::RM_RefereeSystemSetLine((char*)"cd1", 2, 570, 540, 595, 540));
                // // 超电下限位
                // add(RM_RefereeSystem::RM_RefereeSystemSetLine((char*)"cd2", 2, 673, 800, 690, 785));

                // // 小陀螺内圆
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
                // add(RM_RefereeSystem::RM_RefereeSystemSetCircle((char*)"xt_in", 2, 1450, 750, 67));
                // add(RM_RefereeSystem::RM_RefereeSystemSetRectangle((char*)"dp9", 2, 1450 - 20, 750 - 61, 1450 + 20, 750 + 62));

                // RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
                // add(RM_RefereeSystem::RM_RefereeSystemSetArced((char*)"vis", 2, 166, 193, 956, 520, 360, 360));

                // // 发送当前功率(P: 文字标签)
                // RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
                // RM_RefereeSystem::RM_RefereeSystemSetStringSize(15);
                // RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
                // add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr((char*)"P", 1, (char*)"Power:", ZM_of_X, ZM_of_Y));

                is_up_ui = true;
            }

            void setLayer()
            {
                // 瞄准线
                RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
                RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
                RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
            }

            void Init()
            {
                setLayer();
                PitchLine();
            }
    };

}

#endif // !UITASK_HPP