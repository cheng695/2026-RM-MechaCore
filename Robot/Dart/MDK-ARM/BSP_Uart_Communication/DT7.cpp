#include "InHpp.hpp"

/*  =========================== 全局的变量 ===========================  */
DT7UartCom_t DT7UartCom = {};

/*  =========================== 进程的变量 ===========================  */
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

/**
**********************************************************************
* @brief:      	GettMessage: 接收DT7遥控器信息
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于接收DT7遥控器的各种原始串口数据
***********************************************************************
**/
void DT7UartComClass::GetMessage()
{
    // // 将当前接收到的数据写入消息队列（非阻塞）
    // osMessageQueuePut(Queue_DT7ToGimbalHandle, DT7UartCom.ReceiveArr, 0, 0);

    // // 如果有更高优先级的任务被唤醒，则进行任务调度（此处保持占位）
    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    // 可选：增加接收到DT7消息的计数器
    // Report.DT7.Count_Paparazzi++;

    // 将 ReceiveArr 数据复制到 UnpackingArr，以便解包函数能正确解析
    memcpy(UnpackingArr, DT7UartCom.ReceiveArr, sizeof(UnpackingArr));

    // 调用新函数处理错帧问题
    DT7UartComClass::FixFrameError();

    // 将接收到的原始数据进行解包
    DT7UartComClass::Unpacking();

    // 开始通过DMA接收新的数据帧，同时会自动禁用传输完成中断
    HAL_UART_Receive_DMA(&DT7UartHandle, DT7UartCom.ReceiveArr, DT7UartReceiveLength);

    // 禁用DMA半传输中断
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/**
**********************************************************************
* @brief:      	Unpacking: 对接收的原始数据进行解包
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于解析DT7原始串口数据
***********************************************************************
**/
void DT7UartComClass::Unpacking()
{
    rc.ch0 = ((int16_t)UnpackingArr[0] | ((int16_t)UnpackingArr[1] << 8)) & 0x07FF;
    rc.ch1 = (((int16_t)UnpackingArr[1] >> 3) | ((int16_t)UnpackingArr[2] << 5)) & 0x07FF;
    rc.ch2 = (((int16_t)UnpackingArr[2] >> 6) | ((int16_t)UnpackingArr[3] << 2) | ((int16_t)UnpackingArr[4] << 10)) & 0x07FF;
    rc.ch3 = (((int16_t)UnpackingArr[4] >> 1) | ((int16_t)UnpackingArr[5] << 7)) & 0x07FF;
    rc.ch4 = ((int16_t)UnpackingArr[16] | ((int16_t)UnpackingArr[17] << 8)) & 0x07FF;
    rc.s1  = ((UnpackingArr[5] >> 4) & 0x000C) >> 2;
    rc.s2  = ((UnpackingArr[5] >> 4) & 0x0003);
    ///////////////////////////////////////////////////////////
    mouse.x                 = (UnpackingArr[6]) | (UnpackingArr[7] << 8);
    mouse.y                 = (UnpackingArr[8]) | (UnpackingArr[9] << 8);
    mouse.y                 = 0 - mouse.y;
    mouse.z                 = (UnpackingArr[10]) | (UnpackingArr[11] << 8);
    mouse.press_l           = UnpackingArr[12];
    mouse.press_r           = UnpackingArr[13];
    key.Uint8_KeyBoard      = ((int16_t)UnpackingArr[14]);
    key.Uint8_KeyBoard_Next = ((int16_t)UnpackingArr[15]);
    ///////////////////////////////////////////////////////////
    Coord.ch0 = (rc.ch0 - 1024);
    Coord.ch1 = (rc.ch1 - 1024);
    Coord.ch2 = (rc.ch2 - 1024) * (-1);
    Coord.ch3 = (rc.ch3 - 1024);
    ///////////////////////////////////////////////////////////
    Coord.Left_Vx  = (rc.ch2 - 1024);
    Coord.Left_Vy  = (rc.ch3 - 1024);
    Coord.Right_Vx = (rc.ch0 - 1024);
    Coord.Right_Vy = (rc.ch1 - 1024);
    ///////////////////////////////////////////////////////////
    key.W     = (key.Uint8_KeyBoard & 0x01) >> 0;
    key.S     = (key.Uint8_KeyBoard & 0x02) >> 1;
    key.A     = (key.Uint8_KeyBoard & 0x04) >> 2;
    key.D     = (key.Uint8_KeyBoard & 0x08) >> 3;
    key.Shift = (key.Uint8_KeyBoard & 0x10) >> 4;
    key.Ctrl  = (key.Uint8_KeyBoard & 0x20) >> 5;
    key.Q     = (key.Uint8_KeyBoard & 0x40) >> 6;
    key.E     = (key.Uint8_KeyBoard & 0x80) >> 7;
    key.R     = (key.Uint8_KeyBoard_Next & 0x01) >> 0;
    key.F     = (key.Uint8_KeyBoard_Next & 0x02) >> 1;
    key.G     = (key.Uint8_KeyBoard_Next & 0x04) >> 2;
    key.Z     = (key.Uint8_KeyBoard_Next & 0x08) >> 3;
    key.X     = (key.Uint8_KeyBoard_Next & 0x10) >> 4;
    key.C     = (key.Uint8_KeyBoard_Next & 0x20) >> 5;
    key.V     = (key.Uint8_KeyBoard_Next & 0x40) >> 6;
    key.B     = (key.Uint8_KeyBoard_Next & 0x80) >> 7;
}

/**
**********************************************************************
* @brief:      	FixFrameError: 修复串口错帧问题
* @param[in]: 	void
* @retval:      void
* @details:    	该函数用于检测并修复串口数据帧错位问题
*               如果ReceiveArr[17]不为4或者ReceiveArr[12~16]这5位不全为0，则视为错帧，
*               尝试移位至ReceiveArr[17]为4且ReceiveArr[12~16]这5位全为0，
*               如果没有错帧，则不处理。
***********************************************************************
**/
void DT7UartComClass::FixFrameError()
{
    // 判断当前帧是否有效
    bool isFrameValid = (ReceiveArr[17] == 4) &&
                        (ReceiveArr[12] == 0 && ReceiveArr[13] == 0 &&
                         ReceiveArr[14] == 0 && ReceiveArr[15] == 0 && ReceiveArr[16] == 0);

    if (!isFrameValid)
    {
        Dart.isDT7Misaligned = true; // 标记为错帧
        int offset = -1;
        // 查找下一个可能的有效帧起始位置
        for (int i = 0; i <= sizeof(ReceiveArr) - 18; i++)
        {
            if (ReceiveArr[i + 17] == 4 &&
                ReceiveArr[i + 12] == 0 && ReceiveArr[i + 13] == 0 &&
                ReceiveArr[i + 14] == 0 && ReceiveArr[i + 15] == 0 && ReceiveArr[i + 16] == 0)
            {
                offset = i;
                break;
            }
        }

        // 如果找到了有效帧起始位置，则将数据左移对齐
        if (offset != -1 && offset > 0)
        {
            memmove(ReceiveArr, &ReceiveArr[offset], sizeof(ReceiveArr) - offset);
            // 填充无效数据为0x00
            for (int i = sizeof(ReceiveArr) - offset; i < sizeof(ReceiveArr); i++)
            {
                ReceiveArr[i] = 0x00;
            }
        }
        else
        {
            // 如果未找到有效帧头，清空数据
            //memset(ReceiveArr, 0x00, sizeof(ReceiveArr));
        }
    }
}
