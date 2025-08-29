#ifndef __Filter_Hpp
#define __Filter_Hpp
/* C++代码的声明 ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/*Kalman_Begin-----------------------------------------------------------------------------------------------------------------*/
typedef struct 
{
    float X_last;
    float X_mid; 
    float X_now; 
    float P_mid; 
    float P_now;  
    float P_last; 
    float kg;     
    float A;   
    float Q;
    float R;
    float H;
}Kalman_t;
float KalmanFilter(Kalman_t* p,float dat);
void kalmanCreate(Kalman_t *p,float T_Q,float T_R);
/*Kalman_End-----------------------------------------------------------------------------------------------------------------*/



/*TD_Begin-----------------------------------------------------------------------------------------------------------------*/
typedef struct TD_t
{     
	float v1,v2; 
	float R;           
	float H;           
}TD_t;
float TdFilter(TD_t *TD,float Input);
// extern TD_t TD_UpFriction;
// extern TD_t TD_LeftDonwFrictionWheel;
// extern TD_t TD_RIghtDownFrictionWheel;
// extern TD_t TD_LeftFrictionWheel;
// extern TD_t TD_RightFrictionWheel;
// extern TD_t TD_VisionYaw;
// extern TD_t TD_VisionPitch;

extern TD_t TD_RightDownFriction;
extern TD_t TD_RightUpFriction;
extern TD_t TD_LeftUpFriction;
extern TD_t TD_LeftDownFriction;
extern TD_t TD_AngleSensor;

/*TD_End-----------------------------------------------------------------------------------------------------------------*/



/*LPF_Begin-----------------------------------------------------------------------------------------------------------------*/
typedef struct
{
	float Last_Out;
	float Ratio;
}LPF_Data_t;
float LPFFilter(LPF_Data_t *LPF_Data,float Inupt);
/*LPF_End-----------------------------------------------------------------------------------------------------------------*/



/*LimitFilter_Begin-----------------------------------------------------------------------------------------------------------------*/
typedef struct
{
	float Last_Out;
	float Limit_Ratio;
}LMF_Data_t;
float LPFFilter(LPF_Data_t *LPF_Data,float Inupt);
/*LimitFilter_End-----------------------------------------------------------------------------------------------------------------*/
/* USER CODE END Includes */

/* C++代码的声明 ----------------------------------------------------------*/
#endif
