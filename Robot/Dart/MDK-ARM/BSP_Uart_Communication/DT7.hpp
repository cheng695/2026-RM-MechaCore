#ifndef __DT7_Hpp
#define __DT7_Hpp

#define DT7UartHandle        huart1
#define DT7UartInstance      USART1
#define DT7UartReceiveLength 18

typedef class DT7UartComClass
{
public:
    bool isConnected; // 遥控器连接状态
    unsigned char UnpackingArr[DT7UartReceiveLength];
    unsigned char ReceiveArr[DT7UartReceiveLength];
    
    struct
    {
        unsigned short ch4;
        unsigned short ch0;
        unsigned short ch1;
        unsigned short ch2;
        unsigned short ch3;
        unsigned char s1;
        unsigned char s2;
    } rc;
    struct
    {
        short x;
        short y;
        short z;
        unsigned char press_l;
        unsigned char press_r;
    } mouse;
    struct
    {
        unsigned char Uint8_KeyBoard;
        unsigned char Uint8_KeyBoard_Next;
        unsigned char W;
        unsigned char A;
        unsigned char S;
        unsigned char D;
        unsigned char Q;
        unsigned char E;
        unsigned char Shift;
        unsigned char Ctrl;

        unsigned char R;
        unsigned char F;
        unsigned char G;
        unsigned char Z;
        unsigned char X;
        unsigned char C;
        unsigned char V;
        unsigned char B;
    } key;
    struct
    {
        int Left_Vx;
        int Left_Vy;

        int Right_Vx;
        int Right_Vy;

        int ch0;
        int ch1;
        int ch2;
        int ch3;
    } Coord;
    void GetMessage();
    void Unpacking();
    void FixFrameError();
} DT7UartCom_t;
extern DT7UartCom_t DT7UartCom;

#endif
