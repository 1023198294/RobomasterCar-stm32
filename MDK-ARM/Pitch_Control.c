#include "Pitch_Control.h"

#include "stdio.h"

int Pitch_Chassis_Speed = 0 ;
int Pitch_read_ecoder_last = 0 ;
float Pitch_Location_least = 0 ;
float Pitch_Speed_least = 0 ;
int Pitch_except_i = 0 ;
int Pitch_Chassis_least = 0  ,  Pitch_Chassis_PID_diff_feedback = 1 ;
float Pitch_except_v = 0 ;
float outshow_num = 0 , outshow_num1 = 0;
float KalmanFitter3(float MeasVar);
float KalmanFitter4(float MeasVar);

static int16_t Pitch_speed_last_imu_least , Pitch_speed_integral,Pitch_Speed_diff_except,except_v_last,Pitch_Speed_diff_least, Pitch_speed_last_imu;
static float Pitch_Location_diff_feedback=0,read_imu_last=0,Pitch_Location_diff_except=0,except_imu_last=0,Pitch_Location_integral=0;

int New_Pitch_Position(int read_pitch_position , int target_pitch_position){
	static float Pitch_Position_PID_P = 0 , Pitch_Position_PID_I = 0 , Pitch_Position_PID_D = 0 ;
	static int Pitch_Position_least  , Pitch_Position_last_least , Pitch_Position_integral_least  ;
	int Output_position_P = 0 , Output_position_I = 0 ,Output_position_D = 0   , Output_target_speed = 0 ;
	Pitch_Position_least = target_pitch_position - read_pitch_position ;
	Output_position_P = Pitch_Position_PID_P * Pitch_Position_least ;
	Pitch_Position_integral_least += Pitch_Position_least ;
	Output_position_I = Pitch_Position_PID_I * Pitch_Position_integral_least ;
	if(Output_position_I > 1000)Output_position_I = 1000 ;
	else if(Output_position_I < -1000)Output_position_I = -1000;
	Output_position_D = Pitch_Position_PID_D * (Pitch_Position_least - Pitch_Position_last_least);
	Output_target_speed = Output_position_P + Output_position_I + Output_position_D ;
	if(Output_target_speed > 2000)Output_target_speed = 2000 ;
	else if(Output_target_speed < -2000)Output_target_speed = -2000;
	Pitch_Position_last_least = Pitch_Position_least ;
	return Output_target_speed ;
}

int New_Pitch_Speed(int read_pitch_speed , int target_pitch_speed){
	static float Pitch_Speed_PID_P = 0 , Pitch_Speed_PID_I = 0 , Pitch_Speed_PID_D = 0 ;
	static int Pitch_Speed_least  , Pitch_Speed_last_least , Pitch_Speed_integral_least  ;
	int Output_Speed_P = 0 , Output_Speed_I = 0 ,Output_Speed_D = 0   , Output_target_current = 0 ;
	Pitch_Speed_least = target_pitch_speed - read_pitch_speed ;
	Output_Speed_P = Pitch_Speed_PID_P * Pitch_Speed_least ;
	Pitch_Speed_integral_least += Pitch_Speed_least ;
	Output_Speed_I = Pitch_Speed_PID_I * Pitch_Speed_integral_least ;
	if(Output_Speed_I > 1000)Output_Speed_I = 1000 ;
	else if(Output_Speed_I < -1000)Output_Speed_I = -1000;
	Output_Speed_D = Pitch_Speed_PID_D * (Pitch_Speed_least - Pitch_Speed_last_least);
	Output_target_current = Output_Speed_P + Output_Speed_I + Output_Speed_D ;
	if(Output_target_current > 2000)Output_target_current = 2000 ;
	else if(Output_target_current < -2000)Output_target_current = -2000;
	Pitch_Speed_last_least = Pitch_Speed_least ;
	return Output_target_current ;
}

//云台Pitch_位置环
int16_t Pitch_Location(float read_imu ,float except_imu ,  float except_v){
	float PID_P = 5 , PID_I = 0.00, PID_D =100 ,D_feedback=0,D_except=0;
	Pitch_Location_diff_feedback=read_imu-read_imu_last;
	Pitch_Location_diff_except=except_imu-except_imu_last;
	read_imu_last=read_imu; 
	except_imu_last=except_imu;
	Pitch_Location_least = except_imu - (read_imu+D_feedback*KalmanFitter3(Pitch_Location_diff_feedback));
	Pitch_Location_integral+=Pitch_Location_least;
	
	if(Pitch_Location_least>180)Pitch_Location_least-=360;
	if(Pitch_Location_least<-180)Pitch_Location_least+=360;
	outshow_num = Pitch_Location_integral;
	if(Pitch_Location_integral > 700)Pitch_Location_integral=700;
	if(Pitch_Location_integral < -700)Pitch_Location_integral = -700;
	except_v = PID_P*Pitch_Location_least + PID_I*Pitch_Location_integral + PID_D*(read_imu - read_imu_last);
	Pitch_except_v = except_v ;
	if(except_v > 200)except_v= 200;
	if(except_v < -200)except_v = -200 ;
	outshow_num1 = except_v;
	return except_v ;
}

//云台Pitch_速度环
//int16_t Pitch_Speed(float read_imu ,float except_v ,  int except_i){
//	float PID_P = 30  , PID_I = 0, PID_D = 150 ,D_except= 0 ,Pitch_Speed_least_last=0;
//	static int16_t Pitch_speed_last_imu_least = 0 , Pitch_speed_integral=0,Pitch_Speed_diff_except=0,except_v_last=0,Pitch_Speed_diff_least=0 , Pitch_speed_last_imu = 0 ;
//	if(read_imu > 2000)read_imu = read_imu - 4000 ;
//	Pitch_Speed_diff_except=except_v-except_v_last;
//	except_v_last=except_v;
//	Pitch_Speed_least = except_v - read_imu ;
//	Pitch_Speed_least_last=Pitch_Speed_least;
//	Pitch_speed_integral+=Pitch_Speed_least;
//	except_i = PID_P * Pitch_Speed_least + PID_I * Pitch_speed_integral + PID_D*KalmanFitter4(read_imu-Pitch_speed_last_imu)+ D_except*Pitch_Speed_diff_except;
//    Pitch_speed_last_imu_least = Pitch_Speed_least ;
//	Pitch_speed_last_imu = read_imu ;
//	Pitch_except_i = except_i ;
//	if(except_i > 3500)except_i = 3500 ;
//	if(except_i < -3500)except_i = -3500 ;
//	return except_i ;
//}
int16_t Pitch_Speed(float read_imu ,float except_v ,  int except_i){
	float PID_P = 6  , PID_I = 0, PID_D = 100,D_except= 0 ,Pitch_Speed_least_last=0;
	
	if(read_imu > 2000)read_imu = read_imu - 4000 ;
	Pitch_Speed_diff_except=except_v-except_v_last;
	except_v_last=except_v;
	Pitch_Speed_least = except_v - read_imu ;
	Pitch_Speed_least_last=Pitch_Speed_least;
	Pitch_speed_integral+=Pitch_Speed_least;
	
	if(Pitch_speed_integral > 3000)Pitch_speed_integral = 3000;
	else if (Pitch_speed_integral <-3000)Pitch_speed_integral = -3000;
	
	except_i = PID_P * Pitch_Speed_least + PID_I * Pitch_speed_integral + PID_D*KalmanFitter4(read_imu-Pitch_speed_last_imu);
  Pitch_speed_last_imu_least = Pitch_Speed_least ;
	Pitch_speed_last_imu = read_imu ;
	Pitch_except_i = except_i ;
	if(except_i > 3500)except_i = 3500 ;
	if(except_i < -3500)except_i = -3500 ;
	return except_i ;
}

int16_t Pitch_Test(float read_imu ,float except_imu ,  int except_i){
	float PID_P = 300  , PID_I = 0, PID_D = 2800  , except_D = 0;
	static int16_t Pitch_Test_last = 0 , Pitch_Test_least = 0 , Pitch_Test_least_last = 0  , Pitch_except_Least =  0 ;
	Pitch_except_Least = read_imu - Pitch_Test_last;
	Pitch_Test_least = -(except_imu - (read_imu + except_D * KalmanFitter5(Pitch_except_Least))) ;
	except_i = -(PID_P * Pitch_Test_least + PID_D * KalmanFitter7((float)(read_imu - Pitch_Test_last))) ;
	Pitch_Test_last = read_imu ;
	Pitch_Test_least_last = Pitch_Test_least ;
	if(except_i > 3500) except_i = 3500 ;
	if(except_i < -3500) except_i = -3500 ;
	return except_i ;
}

//卡尔曼滤波
#define SysNoiseCoVar         (0.0123)							//系统噪声协方差
#define MeasNoiseCoVar        (16.7415926)          //测量噪声协方差
//SysNoiseCoVar越大，精度越低，速度越快MeasNoiseCovar越大，精度越高，速度越慢
float KalmanFitter3(float MeasVar)
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
float KalmanFitter4(float MeasVar)
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
float KalmanFitter5(float MeasVar)
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
//SysNoiseCoVar越大，精度越低，速度越快MeasNoiseCovar越大，精度越高，速度越慢
float KalmanFitter7(float MeasVar)
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
