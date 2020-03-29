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

//����Yaw_PID
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
//��̨Yaw_λ�û�
int16_t Yaw_head_Location(float read_imu ,float except_imu ,  float except_v){
	float PID_P = 3  , PID_I = 0, PID_D = 200 ,D_feedback=1,D_except=0.5;
	static float yaw_Location_diff_feedback=0,read_imu_last=0,yaw_Location_diff_except=0,except_imu_last=0,yaw_Location_integral=0;
	if(Yaw_init&&Yaw_init2){Yaw_except = except_imu_last=except_imu =read_imu_last=read_imu;if(Yaw_init==2) Yaw_init2 = 0;}//�˾�������z�᲻���ϵ�����������
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

//��̨Yaw_�ٶȻ�
int16_t Yaw_head_Speed(float read_imu ,float except_v ,  int except_i){
	float PID_P = 20  , PID_I = 0, PID_D =10 ,D_except=0;
	//float PID_P = 40  , PID_I = 0, PID_D =10 ,D_except=0;//������1
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
//�������˲�
#define SysNoiseCoVar         (0.0123)							//ϵͳ����Э����
#define MeasNoiseCoVar        (16.7415926)          //��������Э����
//SysNoiseCoVarԽ�󣬾���Խ�ͣ��ٶ�Խ��MeasNoiseCovarԽ�󣬾���Խ�ߣ��ٶ�Խ��
float KalmanFitter(float MeasVar)
{
    static double	AdjustVar1=0;
    static double PreOptimalVar=1;                      //��ǰ����ֵ
    static double CurMeasVar=0,CurEstimateVar=0;        //��ǰ����ֵ����ǰԤ��ֵ
    static double CurOptimalVar=0;                      //��ǰ����ֵ
    static double CurSysCoVar=0,PreSysCoVar=30;         //��ǰϵͳЭ�����ǰϵͳЭ����
    static double KalmanGain=0;                         //����������
    CurMeasVar=MeasVar;                         				//��ǰϵͳ����ֵ
    //��ǰϵͳ����ֵ����ǰϵͳЭ������Ҫ���÷����ʼֵ��������
   
    /*��ǰ����ֵ=��ǰ����ֵ+����ֵ*/
    CurEstimateVar=PreOptimalVar+AdjustVar1;
    AdjustVar1=0;                                //�趨����ֵ��ԭ���Ǽӿ�����ٶ�
  
    /*��ǰϵͳЭ����=��ǰϵͳЭ����+ϵͳ����Э����*/
    CurSysCoVar=PreSysCoVar+SysNoiseCoVar;
   
    /*����������=��ǰϵͳЭ����/(��ǰϵͳЭ����+��������Э����)*/
    KalmanGain=CurSysCoVar/(CurSysCoVar+MeasNoiseCoVar);
   
    /*��ǰϵͳ����ֵ=��ǰϵͳ����ֵ+����������*������ֵ-��ǰϵͳ����ֵ��*/
    CurOptimalVar=CurEstimateVar+KalmanGain*(CurMeasVar-CurEstimateVar);
   
    /*��ǰϵͳ����Э����=��1-���������棩x��ǰϵͳЭ����*/
    PreSysCoVar =(1-KalmanGain)*CurSysCoVar;
   
		/*��ǰϵͳ����ֵ=��ǰϵͳ����ֵ*/
    PreOptimalVar=CurOptimalVar;		
    
    /*��������*/
    return (float)CurOptimalVar;
}

//�������˲�
#define SysNoiseCoVar         (0.0123)							//ϵͳ����Э����
#define MeasNoiseCoVar        (16.7415926)          //��������Э����
//SysNoiseCoVarԽ�󣬾���Խ�ͣ��ٶ�Խ��MeasNoiseCovarԽ�󣬾���Խ�ߣ��ٶ�Խ��
float KalmanFitter2(float MeasVar)
{
    static double	AdjustVar1=0;
    static double PreOptimalVar=1;                      //��ǰ����ֵ
    static double CurMeasVar=0,CurEstimateVar=0;        //��ǰ����ֵ����ǰԤ��ֵ
    static double CurOptimalVar=0;                      //��ǰ����ֵ
    static double CurSysCoVar=0,PreSysCoVar=30;         //��ǰϵͳЭ�����ǰϵͳЭ����
    static double KalmanGain=0;                         //����������
    CurMeasVar=MeasVar;                         				//��ǰϵͳ����ֵ
    //��ǰϵͳ����ֵ����ǰϵͳЭ������Ҫ���÷����ʼֵ��������
   
    /*��ǰ����ֵ=��ǰ����ֵ+����ֵ*/
    CurEstimateVar=PreOptimalVar+AdjustVar1;
    AdjustVar1=0;                                //�趨����ֵ��ԭ���Ǽӿ�����ٶ�
  
    /*��ǰϵͳЭ����=��ǰϵͳЭ����+ϵͳ����Э����*/
    CurSysCoVar=PreSysCoVar+SysNoiseCoVar;
   
    /*����������=��ǰϵͳЭ����/(��ǰϵͳЭ����+��������Э����)*/
    KalmanGain=CurSysCoVar/(CurSysCoVar+MeasNoiseCoVar);
   
    /*��ǰϵͳ����ֵ=��ǰϵͳ����ֵ+����������*������ֵ-��ǰϵͳ����ֵ��*/
    CurOptimalVar=CurEstimateVar+KalmanGain*(CurMeasVar-CurEstimateVar);
   
    /*��ǰϵͳ����Э����=��1-���������棩x��ǰϵͳЭ����*/
    PreSysCoVar =(1-KalmanGain)*CurSysCoVar;
   
		/*��ǰϵͳ����ֵ=��ǰϵͳ����ֵ*/
    PreOptimalVar=CurOptimalVar;		
    
    /*��������*/
    return (float)CurOptimalVar;
}

