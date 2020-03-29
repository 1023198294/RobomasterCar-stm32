/**
  *@file Yaw_Contral.c
  *@date 2017-5-10
  *@author Jzhiyshen
  *@brief 
  */
  
#include "Yaw_Contral.h"
#include "stdio.h"
#include "DataScope_DP.h"

int16_t Yaw_Chassis_Speed = 0 ;
int16_t read_ecoder_last = 0 ;
int16_t Yaw_Location_least = 0 ;
int16_t Yaw_Speed_least = 0 ;
int Yaw_except_i = 0 ;
int16_t Yaw_Chassis_least = 0  ,  Yaw_Chassis_PID_diff_feedback = 1 ;
int16_t Yaw_except_v = 0 ;
float KalmanFitter(float MeasVar);
float KalmanFitter2(float MeasVar);

//底盘Yaw_PID
int16_t Yaw_Chassis(int16_t read_ecoder ,int16_t except_ecoder , int16_t speed){
	float PID_P = 200  , PID_I = 0 , PID_D_feedback = 0.1 ;
	//Yaw_Chassis_PID_diff_feedback = read_ecoder - read_ecoder_last;
	//Yaw_Chassis_PID_diff_feedback=KalmanFitter(Yaw_Chassis_PID_diff_feedback);
	//Yaw_Chassis_least = except_ecoder - (read_ecoder+PID_D_feedback*Yaw_Chassis_PID_diff_feedback) ;
    //least = except_ecoder - read_ecoder ;
	//speed = PID_P * Yaw_Chassis_least + PID_D*(read_ecoder - read_ecoder_last);
	
	Yaw_Chassis_least = except_ecoder - read_ecoder;
	if(Yaw_Chassis_least > 150) Yaw_Chassis_least = except_ecoder - (409.0 + read_ecoder);
	speed = PID_P * Yaw_Chassis_least;
	read_ecoder_last=read_ecoder;
	if(speed > 4000) speed = 4000 ;
	if(speed < -4000) speed = -4000 ;
	return speed ;
}
extern int16_t Yaw_except;
int Yaw_init = 1,Yaw_init2 = 1;
//云台Yaw_位置环
int16_t Yaw_head_Location(float read_imu ,float except_imu ,  float except_v){
	float PID_P = 3  , PID_I = 0, PID_D = 200 ,D_feedback=1,D_except=0.5;
	static float yaw_Location_diff_feedback=0,read_imu_last=0,yaw_Location_diff_except=0,except_imu_last=0,yaw_Location_integral=0;
	if(Yaw_init&&Yaw_init2){Yaw_except = except_imu_last=except_imu =read_imu_last=read_imu;if(Yaw_init==2) Yaw_init2 = 0;}//此句代码针对z轴不能上电归零的陀螺仪
	yaw_Location_diff_feedback=read_imu-read_imu_last;
	yaw_Location_diff_except=except_imu-except_imu_last;
	read_imu_last=read_imu; 
	except_imu_last=except_imu;
	Yaw_Location_least = except_imu - (read_imu+D_feedback*KalmanFitter2(yaw_Location_diff_feedback));
	yaw_Location_integral+=Yaw_Location_least;
	if(Yaw_Location_least>180)Yaw_Location_least-=360;
	if(Yaw_Location_least<-180)Yaw_Location_least+=360;
	except_v = PID_P*Yaw_Location_least + PID_I*yaw_Location_integral +D_except*yaw_Location_diff_except;
	Yaw_except_v = except_v ;
	if(except_v > 300)except_v= 300 ;
	if(except_v < -300)except_v = -300 ;
	return except_v ;
}

//云台Yaw_速度环
int16_t Yaw_head_Speed(float read_imu ,float except_v ,  int except_i){
	float PID_P = 20  , PID_I = 0, PID_D =10 ,D_except=0;
	//float PID_P = 40  , PID_I = 0, PID_D =10 ,D_except=0;//步兵车1
	static int16_t yaw_speed_last_imu_least = 0 , yaw_speed_integral=0,yaw_Speed_diff_except=0,except_v_last=0;
	if(read_imu > 2000)read_imu = read_imu - 4000 ;
	yaw_Speed_diff_except=except_v-except_v_last;
	except_v_last=except_v;
	Yaw_Speed_least = except_v - read_imu ;
	yaw_speed_integral+=Yaw_Speed_least;
//	except_i = PID_P * Yaw_Speed_least + PID_D * (Yaw_Speed_least - yaw_speed_last_imu_least);
	except_i = PID_P * Yaw_Speed_least + PID_I * yaw_speed_integral + D_except*yaw_Speed_diff_except;
    yaw_speed_last_imu_least = Yaw_Speed_least ;
	Yaw_except_i = except_i ;
	if(except_i > 3000)except_i = 3000 ;
	if(except_i < -3000)except_i = -3000 ;
	return except_i ;
}
//卡尔曼滤波
#define SysNoiseCoVar         (0.0123)							//系统噪声协方差
#define MeasNoiseCoVar        (16.7415926)          //测量噪声协方差
//SysNoiseCoVar越大，精度越低，速度越快MeasNoiseCovar越大，精度越高，速度越慢
float KalmanFitter(float MeasVar)
{
    static double	AdjustVar1=0;
    static double PreOptimalVar=1;                      //先前最优值
    static double CurMeasVar=0,CurEstimateVar=0;        //当前测量值，当前预测值
    static double CurOptimalVar=0;                      //当前最优值
    static double CurSysCoVar=0,PreSysCoVar=30;         //当前系统协方差，先前系统协方差
    static double KalmanGain=0;                         //卡尔曼增益
    CurMeasVar=MeasVar;                         				//当前系统测量值
    //先前系统最优值和先前系统协方差需要设置非零初始值；；；；
   
    /*当前估计值=先前最优值+调整值*/
    CurEstimateVar=PreOptimalVar+AdjustVar1;
    AdjustVar1=0;                                //设定调整值的原因是加快调整速度
  
    /*当前系统协方差=先前系统协方差+系统噪声协方差*/
    CurSysCoVar=PreSysCoVar+SysNoiseCoVar;
   
    /*卡尔曼增益=当前系统协方差/(当前系统协方差+测量噪声协方差)*/
    KalmanGain=CurSysCoVar/(CurSysCoVar+MeasNoiseCoVar);
   
    /*当前系统最优值=当前系统估计值+卡尔曼增益*（测量值-当前系统估计值）*/
    CurOptimalVar=CurEstimateVar+KalmanGain*(CurMeasVar-CurEstimateVar);
   
    /*先前系统噪声协方差=（1-卡尔曼增益）x当前系统协方差*/
    PreSysCoVar =(1-KalmanGain)*CurSysCoVar;
   
		/*先前系统最优值=当前系统最优值*/
    PreOptimalVar=CurOptimalVar;		
    
    /*返回数据*/
    return (float)CurOptimalVar;
}

//卡尔曼滤波
#define SysNoiseCoVar         (0.0123)							//系统噪声协方差
#define MeasNoiseCoVar        (16.7415926)          //测量噪声协方差
//SysNoiseCoVar越大，精度越低，速度越快MeasNoiseCovar越大，精度越高，速度越慢
float KalmanFitter2(float MeasVar)
{
    static double	AdjustVar1=0;
    static double PreOptimalVar=1;                      //先前最优值
    static double CurMeasVar=0,CurEstimateVar=0;        //当前测量值，当前预测值
    static double CurOptimalVar=0;                      //当前最优值
    static double CurSysCoVar=0,PreSysCoVar=30;         //当前系统协方差，先前系统协方差
    static double KalmanGain=0;                         //卡尔曼增益
    CurMeasVar=MeasVar;                         				//当前系统测量值
    //先前系统最优值和先前系统协方差需要设置非零初始值；；；；
   
    /*当前估计值=先前最优值+调整值*/
    CurEstimateVar=PreOptimalVar+AdjustVar1;
    AdjustVar1=0;                                //设定调整值的原因是加快调整速度
  
    /*当前系统协方差=先前系统协方差+系统噪声协方差*/
    CurSysCoVar=PreSysCoVar+SysNoiseCoVar;
   
    /*卡尔曼增益=当前系统协方差/(当前系统协方差+测量噪声协方差)*/
    KalmanGain=CurSysCoVar/(CurSysCoVar+MeasNoiseCoVar);
   
    /*当前系统最优值=当前系统估计值+卡尔曼增益*（测量值-当前系统估计值）*/
    CurOptimalVar=CurEstimateVar+KalmanGain*(CurMeasVar-CurEstimateVar);
   
    /*先前系统噪声协方差=（1-卡尔曼增益）x当前系统协方差*/
    PreSysCoVar =(1-KalmanGain)*CurSysCoVar;
   
		/*先前系统最优值=当前系统最优值*/
    PreOptimalVar=CurOptimalVar;		
    
    /*返回数据*/
    return (float)CurOptimalVar;
}

