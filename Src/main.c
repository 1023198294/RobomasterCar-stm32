/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "test_beep.h"
#include "stdio.h"
/* USER CODE BEGIN Includes */
#include "test_imu.h"
#include "test_app.h"
#include "test_can.h"
#include "test_uart.h"
#include "Yaw_Contral.h"
#include "Pitch_Control.h"
#include "DataScope_DP.h"
//遥控器参数
int ch0=0 , ch1=0 , ch2=0 , ch3=0 , s1=0 , s2=0  ; //遥控器拨杆
int16_t MouseX = 0 , MouseY = 0 , MouseZ = 0 ;    //鼠标
uint8_t MouseLeft = 0 , MouseRight = 0 ;  //鼠标左右键
uint16_t key_All = 0 , key_W = 0 , key_S = 0 , key_A = 0 , 
		key_D = 0 , key_Q = 0 , key_E = 0 , key_Shift = 0 , key_Ctrl = 0 ;    //键盘
int16_t front_back = 0 , left_right = 0 ;   //键盘控制的速度
int Dbus = 0 ; 
int NUM1=0 , NUM2=0 , NUM3=0 , NUM4  = 0  , NUM5 = 0 , NUM6 = 0 , NUM7 = 0 , NUM8 = 0 ;    //电流值
int Speed1=0 , Speed2=0 , Speed3=0 , Speed4=0 ;      //目标速度
int16_t ecode1 = 0 , ecode2 = 0 , ecode3 = 0 , ecode4 = 0 ,ecode5 = 0 , ecode5_speed=0,ecode6 = 0 ,ecode6_speed=0,ecode7 = 0 , ecode7_speed = 0,ecode8_speed = 0;  //编码值
int err[2]={0,0};
unsigned char Tower_flag1,Tower_flag2;
//static int ecode_pitch_speed  , ecode_pitch_last_speed ;
float Pitch = 0 ;              //云台Pitch值
float Pitch_Max = 90 ;              //Pitch最大值
float Pitch_Min = -90 ;               //Pitch最小值,原来
int16_t Test_Pitch = 720  , Test_Pitch_Default = 720 ;
int16_t Test_Pitch_Min = 400 ;
int16_t Test_Pitch_Max = 1100 ;
int16_t Pitch_Except_I = 0 ;
int16_t Mouse_Right_Flag = 0 ;
int8_t Pitch_Init_Flag = 0 ;
int8_t sendcount,haha;
//步兵1云台
//#define Yaw_const 221 				//云台Yaw值
//float Yaw = Yaw_const; 
//float Yaw_Max = Yaw_const+65;           //Yaw最大值
//float Yaw_Min = Yaw_const-65;         //Yaw最小值
//extern float outshow_num, outshow_num1, Pitch_Location_least , Pitch_Speed_least , Pitch_except_v;
//////步兵5云台
#define Yaw_const 143 				//云台Yaw值
int16_t Yaw = Yaw_const; 
int16_t Yaw_Max = Yaw_const+35;           //Yaw最大值
int16_t Yaw_Min = Yaw_const-35;         //Yaw最小值
//英雄车云台r
//#define Yaw_const 223 				//云台Yaw值
//int16_t Yaw = Yaw_const; 
//int16_t Yaw_Max = Yaw_const+35;           //Yaw最大值
//int16_t Yaw_Min = Yaw_const-35;         //Yaw最小值

int16_t Yaw_except = 0 ;
int16_t Chassis_Max_I = 8000 ;//4000
int16_t Chassis_Max_I_Default = 4000  , Chassis_Max_I_Default_More = 5000 ;
float test_P;
uint8_t Yaw_except_flag = 0  , Yaw_Chassis_flag = 0  , Yaw_Chassis_auto_flag = 0  , Pitch_except_flag = 0 ;
extern float Read_Pitch, Read_GYaw, Read_Roll , Read_wx , Read_wy , Read_wz ;//陀螺仪数值
extern int16_t Yaw_Chassis_Speed ;
extern int16_t Yaw_Location_least , Yaw_Speed_least , Yaw_except_i ;
extern int16_t Yaw_Chassis_least , Yaw_Chassis_PID_diff_feedback  , Yaw_except_v  , Pitch_except_i ;
/****************英雄车云台参数*****************/
//int16_t Pitch_Hero = 3720 ;
//int16_t Pitch_Max_Hero = 4270 ;
//int16_t Pitch_Min_Hero = 3070 ;
/***********************************************/
//static float least=0 , Integral_least=0 , Last_least=0 ;  //云台PID差积分微分值
static float Speed_Last_Least[5] = {0 , 0 , 0 , 0, 0};
//static float Yaw_least=0 , Yaw_Integral_least=0 , Yaw_Last_least=0 ;  //云台PID差积分微分值
/*****************陀螺仪数据******************/
uint8_t mpu_buff[20];    
IMUDataTypedef imu_data = {0,0,0,0,0,0,0,0,0,0};
IMUDataTypedef imu_data_offest = {0,0,0,0,0,0,0,0,0,0};
/******************************************/
#define BUFFER_SIZE 18
uint8_t rx_len=0;
uint8_t recv_end_flag=0;
	int Yaw_DiffValue[5],Yaw_IntegralValue[5];
uint8_t rx_buffer[18];
//extern uint8_t aRxMessage[50];
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void tim122_pwm_setvalue(uint16_t value);//设置摩擦轮
void tim121_pwm_setvalue(uint16_t value);
void bigshoot1_pwm_setvalue(uint16_t value);//英雄车
void bigshoot2_pwm_setvalue(uint16_t value);
void ReadContral(void);//读遥控器
void ReadEncode(void);//读编码器
void Contral(void);//控制
int PitchPID(int pecode , int pexcept , int pnum);//云台Pitch PID
int YawPID(int yecode , int yexcept , int ynum);//云台Yaw  PID
int SpeedPID(int encoder , int except , int electric , int which);  // 底盘PID
void SetShoot(int shoot);//摩擦轮斜坡函数
void user_pwm_setvalue(uint16_t value) ;
void ShowDP(float var1 ,float var2 ,float var3 , uint8_t showPDs);
int BulletPID(int n , int n0 , int now_e,char number);
int Yaw_Location_PID_Right(int16_t R_value,int16_t C_value);
int Yaw_Speed_PID_Left(int16_t R_value,int16_t C_value);
int Pitch_Location_PID_Right(int16_t R_value,int16_t C_value);
int Pitch_Speed_PID_Left(int16_t R_value,int16_t C_value);
int xa,xb,xc,xd;
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

int NUM7_8_value = 0;
void air_operated_Init()
{
	 GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
	//PC0救推，PB0救压
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
}



/* USER CODE END 0 */



int main(void)
{
	//uint8_t half_falg=0,x=0;
	//int run_sign = 0;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  xa = -100;
	xb = 200;
	xc = 300;
	xd = 400;
	HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
	
	air_operated_Init();
  
  /* USER CODE BEGIN 2 */
  //MPU6500_Init();
  //IST8310_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  
  
  CanFilter_Init(&hcan1);
  CanFilter_Init(&hcan2);
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
  
  HAL_UART_Receive_IT(&huart2, uart2_rx_buff, 1);
  HAL_UART_Receive_IT(&huart3, uart3_rx_buff, 1);
  HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);
  
	
	
  //if(MPU_id != 0)sTestResult.imuTest = 0x01;
  /* USER CODE END 2 */

  /* Infinite loop */
  //摩擦轮初始化
  //HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	
	//tim121_pwm_setvalue(0xFFFF);
 // tim122_pwm_setvalue(0xFFFF);
  
	//tim121_pwm_setvalue(0);
 // tim122_pwm_setvalue(0);
	
  //tim121_pwm_setvalue(833);
  //tim122_pwm_setvalue(833);
	//user_pwm_setvalue(2000);

//	TIM8->CCR3 = 1000;
//	TIM8->CCR4 = 1000;
	
	//user_pwm_setvalue(1000);
	
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);    //使能空闲中断
  //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);//取弹气缸1撑腿
  //HAL_Delay(200);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);//取弹气缸1缩腿
  while (1)
  {
	

			//		second_bodan_count++;
//		if(second_bodan_count>=40000)
//			second_bodan_count=0;
		//printf("%d  %d    %d\r\n",s1,s2,ecode7_speed);
	
			//printf("%d , %d ,%d , %d\n",xa,xb,xc,xd);
			HAL_Delay(50);
		 //printf("%d   \r\n",ch2);
		 //printf("%f , %f\r\n",Read_GYaw,Read_Pitch);//
		 //printf("%d \r\n",ecode5);
		//HAL_Delay(1);
		//Contral();
		
		//printf("%c\n",USART_RX_BUF[3]);
		//printf("%d\n",text_cx);
		//if(text_cx>100)
			//text_cx=0;
			
		}
		
		

      /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

}

//读取遥控器参数
void ReadContral(){
	if(recv_end_flag ==1){  
		//printf("rx_len=%d\r\n",rx_len);  		
		ch0 = (rx_buffer[0]| (rx_buffer[1] << 8)) & 0x07ff;
		ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff;
		ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;
		ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff; 
		s1 = ((rx_buffer[5] >> 4)& 0x000C) >> 2; 
		s2 = ((rx_buffer[5] >> 4)& 0x0003); 
		MouseX = rx_buffer[6] | (rx_buffer[7] << 8); 
		MouseY = rx_buffer[8] | (rx_buffer[9] << 8); 
		MouseZ = rx_buffer[10] | (rx_buffer[11] << 8); 
		MouseLeft = rx_buffer[12]; 
		MouseRight = rx_buffer[13];
		key_All = rx_buffer[14] | (rx_buffer[15] << 8); 
		key_W = key_All & 1 ;
		key_S = (key_All & (1 << 1 ))>>1;
		key_A = (key_All & (1 << 2 ))>>2;
		key_D = (key_All & (1 << 3 ))>>3;
		key_Shift = (key_All & (1 << 4 ))>>4;
		key_Ctrl = (key_All & (1 << 5 ))>>5;
		key_Q = (key_All & (1 << 6 ))>>6;
		key_E = (key_All & (1 << 7 ))>>7;
		//printf("%d %d %d %d %d %d \n" , ch0 , ch1 , ch2 , ch3 , s1 , s2);
		//printf("%d %d %d %d %d %d \n" , MouseX , MouseY , MouseZ , MouseLeft , MouseRight , key_All);
		//printf("%d %d %d %d %d %d %d %d \n" , key_W , key_S , key_A , key_D , key_Shift , key_Ctrl ,  key_Q , key_E );
		rx_len=0;
		recv_end_flag=0;
		//检测遥控器是否开启
		if((ch0 != 0 ) && (ch1 != 0 )&&(ch2 != 0 ) && (ch3 != 0 )){
			Dbus = 1 ;
		}else{
			Dbus = 0 ;
		}
	}
	HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);
}
//读编码器的值
void ReadEncode(){
	ecode1 = (can1_rx_data_buf[0][2]<<8) | can1_rx_data_buf[0][3] ;
	ecode2 = (can1_rx_data_buf[1][2]<<8) | can1_rx_data_buf[1][3] ;
	ecode3 = (can1_rx_data_buf[2][2]<<8) | can1_rx_data_buf[2][3] ;
	ecode4 = (can1_rx_data_buf[3][2]<<8) | can1_rx_data_buf[3][3] ;
	
	ecode5 = ((can1_rx_data_buf[4][0]<<8) | can1_rx_data_buf[4][1]);//数值减小20倍
	ecode5_speed= ((can1_rx_data_buf[4][2]<<8) | can1_rx_data_buf[4][3]);
  ecode6 = ((can1_rx_data_buf[5][0]<<8) | can1_rx_data_buf[5][1]);//数值减小20倍
	ecode6_speed = ((can1_rx_data_buf[5][2]<<8) | can1_rx_data_buf[5][3]);
	//ecode7 = ((can1_rx_data_buf[6][0]<<8) | can1_rx_data_buf[6][1]);
	//ecode7_speed = ((can1_rx_data_buf[6][2]<<8) | can1_rx_data_buf[6][3]);
	//ecode8_speed = ((can1_rx_data_buf[7][2]<<8) | can1_rx_data_buf[7][3]);
if(ecode1 == -3) ecode1 = 0 ;
if(ecode2 == -3) ecode2 = 0 ;
if(ecode3 == -3) ecode3 = 0 ;
if(ecode4 == -3) ecode4 = 0 ;

	
	//printf("ecode : %d  %d  %d  %d  %d  %d  %d  %d \n" , ecode1 , ecode2 , ecode3 , ecode4 , ecode5 , ecode6 , ecode7 , ecode8);
}
void Contral(){
	if(Dbus == 1){
/****************************可左右平移，上下俯仰，前进后退***********************************/
		if (s2 == 3)
			{
//控制俯仰的Pitch电机ID设置成1,Yaw设置成2
				if(Pitch_except_flag < 25){
				Pitch_except_flag++; 
			}else{
				Pitch_except_flag = 0 ;
				Pitch += ( 1024 - ch3 )/50;
				if(Pitch <0)Pitch += 360;
				if(Pitch >= 360)Pitch -= 360 ;
				if(Pitch < 100)Pitch = 100;//限位，限制角度成100-300度
				if(Pitch > 300)Pitch = 300 ;//限位，限制角度成100-300度
				Pitch_except_flag = 0 ;
			}
			
	    	ecode5=(ecode5/22.75);//转换成0-360
		   // Pitch=(360-Pitch);   //加这句可让遥控控制方向与原来方向反过来
		    NUM5 = Pitch_Speed_PID_Left(Pitch_Location_PID_Right(Pitch,ecode5),ecode5_speed);

				
				if(Yaw_except_flag < 25){
				Yaw_except_flag++; 
			}else{
				 
				Yaw_except += ( 1024 - ch2 )/50;
				if(Yaw_except < 0 )Yaw_except += 360;
				if(Yaw_except >= 360)Yaw_except -= 360 ;
						
				if(Yaw_except <100 )Yaw_except =100;//限位，限制角度成100-300度
   			if(Yaw_except >300)Yaw_except = 300 ;	//限位，限制角度成100-300度
				Yaw_except_flag = 0 ;
			}		
		  ecode6=(ecode6/22.75);
	    Yaw=Yaw_except;
	//	  Yaw=(360-Yaw);//加这句可让遥控控制方向与原来方向反过来
		  NUM6 = Yaw_Speed_PID_Left(Yaw_Location_PID_Right(Yaw,ecode6),ecode6_speed);
				
				
			Speed1= ((1024 - ch1) + -(1024 - ch0))*12;
			Speed2 = -((1024 - ch1) + (1024 - ch0))*12;
			Speed3 = ((1024 - ch1) + (1024 - ch0))*12; 
			Speed4 = -((1024 - ch1) + -(1024 - ch0))*12; 
		  }
/*****************************普通模式云台Yaw轴不动，底盘可旋转，云台Pitch可俯仰************************************/
		else if(s2 == 1){
			
			
								if(Pitch_except_flag < 25){
				Pitch_except_flag++; 
			}else{
				Pitch_except_flag = 0 ;
				Pitch += ( 1024 - ch3 )/50;
				if(Pitch <0)Pitch += 360;
				if(Pitch >= 360)Pitch -= 360 ;
				
				if(Pitch < 100)Pitch = 100;//限位，限制角度成100-300度
				if(Pitch > 300)Pitch = 300 ;//限位，限制角度成100-300度
				Pitch_except_flag = 0 ;
			}
			
	    	ecode5=(ecode5/22.75);//转换成0-360
		   // Pitch=(360-Pitch);   //加这句可让遥控控制方向与原来方向反过来
		    NUM5 = Pitch_Speed_PID_Left(Pitch_Location_PID_Right(Pitch,ecode5),ecode5_speed);
			
			Yaw_Chassis_Speed =( 1024 - ch2 )*6;
			Speed1= ((1024 - ch1) + -(1024 - ch0))*12 -Yaw_Chassis_Speed;
			Speed2 = -((1024 - ch1) + (1024 - ch0))*12 -Yaw_Chassis_Speed;
			Speed3 = ((1024 - ch1) + (1024 - ch0))*12 -Yaw_Chassis_Speed;
			Speed4 = -((1024 - ch1) + -(1024 - ch0))*12 -Yaw_Chassis_Speed;
			
		   
			
			
		}
/******************************底盘电机全部断电************************************/
		else if(s2 == 2){
			
			NUM1=NUM2=NUM3=NUM4=NUM5=NUM6=0;
			

		}
		NUM1 = SpeedPID(ecode1 , Speed1 , NUM1 , 0);
		NUM2 = SpeedPID(ecode2 , Speed2 , NUM2 , 1);
		NUM3 = SpeedPID(ecode3 , Speed3 , NUM3 , 2);
		NUM4 = SpeedPID(ecode4 , Speed4 , NUM4 , 3);
		
		
	}
	else{
		NUM1 = 0 ; NUM2 = 0 ; NUM3 = 0 ; 
		NUM4 = 0 ; NUM5 = 0;NUM6 = 0;NUM7 = 0;NUM8 = 0;

	}
}

int Yaw_Location_PID_Right(int16_t R_value,int16_t C_value)
{
	float P = 2, I = 0.002, D = 0;
	float C_new=0,C_old=0,C_miss=0;
	int CurrentValue=0;
	C_old = Yaw_DiffValue[1];
	C_new = R_value - C_value;
	if(C_new>180)C_new-=360;
	if(C_new<(-180))C_new+=360;
	C_miss = C_new - C_old;
	Yaw_DiffValue[1] = C_new;
	Yaw_IntegralValue[1] += Yaw_DiffValue[1];
	if(Yaw_IntegralValue[1] > 3000)Yaw_IntegralValue[1] = 3000;
	else if (Yaw_IntegralValue[1] <-3000)Yaw_IntegralValue[1] = -3000;
	CurrentValue = (int)((P*Yaw_DiffValue[1]) + (I*Yaw_IntegralValue[1]) + (D*C_miss));
	if(CurrentValue > 100) CurrentValue = 100;
	if(CurrentValue < -100) CurrentValue = -100;
	return CurrentValue;
}

int Pitch_Location_PID_Right(int16_t R_value,int16_t C_value)
{
	float P = 2, I = 0.002, D = 0;
	float C_new=0,C_old=0,C_miss=0;
	int CurrentValue=0;
	C_old = Yaw_DiffValue[2];
	C_new = R_value - C_value;
	if(C_new>180)C_new-=360;
	if(C_new<(-180))C_new+=360;
	C_miss = C_new - C_old;
	Yaw_DiffValue[2] = C_new;
	Yaw_IntegralValue[2] += Yaw_DiffValue[2];
	if(Yaw_IntegralValue[2] > 3000)Yaw_IntegralValue[2] = 3000;
	else if (Yaw_IntegralValue[2] <-3000)Yaw_IntegralValue[2] = -3000;
	CurrentValue = (int)((P*Yaw_DiffValue[2]) + (I*Yaw_IntegralValue[2]) + (D*C_miss));
	if(CurrentValue > 100) CurrentValue = 100;
	if(CurrentValue < -100) CurrentValue = -100;
	return CurrentValue;
}


int Yaw_Speed_PID_Left(int16_t R_value,int16_t C_value)
{
	float P = 56, I = 0.2, D = 0;
	float C_new=0,C_old=0,C_miss=0;
	int CurrentValue=0;
	C_old = Yaw_DiffValue[4];
	C_new = R_value - C_value;
	C_miss = C_new - C_old;
	Yaw_DiffValue[4] = R_value - C_value;
	Yaw_IntegralValue[4] += Yaw_DiffValue[4];
	CurrentValue = (int)((P*Yaw_DiffValue[4]) + (I*Yaw_IntegralValue[4]) + (D*C_miss));
	if(CurrentValue > 12000) CurrentValue = 12000;
	if(CurrentValue < -12000) CurrentValue = -12000;
	return CurrentValue;
}

int Pitch_Speed_PID_Left(int16_t R_value,int16_t C_value)
{
	float P = 56, I = 0.2, D = 0;
	float C_new=0,C_old=0,C_miss=0;
	int CurrentValue=0;
	C_old = Yaw_DiffValue[3];
	C_new = R_value - C_value;
	C_miss = C_new - C_old;
	Yaw_DiffValue[3] = R_value - C_value;
	Yaw_IntegralValue[3] += Yaw_DiffValue[3];
	CurrentValue = (int)((P*Yaw_DiffValue[3]) + (I*Yaw_IntegralValue[3]) + (D*C_miss));
	if(CurrentValue > 12000) CurrentValue = 12000;
	if(CurrentValue < -12000) CurrentValue = -12000;
	return CurrentValue;
}

//拨弹轮PID
int BulletPID(int n , int n0 , int now_e,char number)
{
	float P = 1 ,D = 50;
	int temp,miss;
	temp = err[number-7];
	err[number-7] = n0 - n;	
	miss = err[number-7] - temp;
	temp = now_e + P*err[number-7] + D*miss;
	
	if(temp > 5000) temp = 5000;
	if (temp < -5000) temp = -5000;
  return temp;	
}






//底盘PID
int SpeedPID(int encoder , int except , int electric , int which){
	float PID_P = 0.03   , PID_D = 3 ;
	float least = 0  , new_electric = 0 ;
	least = except - encoder ;
	new_electric = electric + least*PID_P + PID_D*(least - Speed_Last_Least[which]) ;
	Speed_Last_Least[which] = least ;
	if(new_electric > Chassis_Max_I) new_electric = Chassis_Max_I ;
	if(new_electric < -Chassis_Max_I) new_electric = -Chassis_Max_I ;
	return new_electric;
}

void tim121_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);  
}
void tim122_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);  
}

void user_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;	
	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); 
	
    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);  
}


/*****************************End*****************************************/
















































//电脑上位机显示
//void ShowDP(float var1 ,float var2 ,float var3 , uint8_t showPDs){
//	static int showPD_flag = 0 ;//显示分频
//	if(showPD_flag < showPDs){
//		showPD_flag++ ;
//		return ;
//	}else{
//		showPD_flag = 0 ;
//	}
//	int Send_Count = 0 ;
//	DataScope_Get_Channel_Data((float)var1, 1 );
//	DataScope_Get_Channel_Data((float)var2, 2 );
//	DataScope_Get_Channel_Data((float)var3, 3 );
//	Send_Count = DataScope_Data_Generate(3);
//	HAL_UART_Transmit(&huart6, (uint8_t *)&DataScope_OutPut_Buffer, Send_Count, 0xFFFF);
//}



////云台Pitch PID
//int PitchPID(int pecode , int pexcept , int pnum){
//	float pnum_temp,pnum_lost=0;
//	float PID_P = 5  ,PID_I = 0, PID_D = 50 ;
//	least = pexcept - pecode ;              //求偏差
//	Integral_least += least ;
//	if(1){ PID_I = 0 , Integral_least=0 ,PID_P = 3 ,PID_D = 50;}	//积分分离,带死区
//	else{ PID_I = 0.01, PID_P = 4 , PID_D = 100 ;}
//	pnum_temp = least*PID_P  + (float)PID_I*Integral_least + (float)PID_D*(least - Last_least);
//	
//	if(1)pnum_lost=0.8*(pnum+pnum_lost)+0.2*pnum_temp;
//	else pnum_lost=0.7*(pnum+pnum_lost)+0.3*pnum_temp;
//	
//	Last_least = least ;
//	pnum=pnum_lost;
//	pnum_lost-=pnum;
//	
//	if(pnum > 3500) pnum = 3500 ;                 //限制输出最大值
//	if(pnum < -3500) pnum = -3500 ;               //限制输出最小值
//	//printf("%d\n" , ynum);
//	return pnum ;
//}
////云台Yaw PID
//int YawPID(int yecode , int yexcept , int ynum){
//	//float ynum_temp,ynum_lost=0;
//	float PID_P = 2  ,PID_I = 0, PID_D = 10 ;
//	Yaw_least = yexcept - yecode ;              //求偏差
//	Yaw_Integral_least += Yaw_least ;
//	ynum = Yaw_least * PID_P + Yaw_Integral_least * PID_I + PID_D*(Yaw_least - Yaw_Last_least);
//	Yaw_Last_least = Yaw_least ;
//	if(ynum > 3500) ynum = 3500 ;                 //限制输出最大值
//	if(ynum < -3500) ynum = -3500 ;               //限制输出最小值
//	printf("%d\n" , ynum);
//	return ynum ;
//}








//void SetShoot(int shoot){
//	static float pwm_shoot = 833 ;
//	if(shoot == 1 && pwm_shoot >= 1666){
//		pwm_shoot = 1666 ;
//	}else{
//		pwm_shoot = pwm_shoot + 0.2;
//	};
//	if(shoot == 0){
//		pwm_shoot = 833 ;
//	}
//	tim121_pwm_setvalue((int)pwm_shoot);
//	tim122_pwm_setvalue((int)pwm_shoot);
//}




/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
 
}

/* USER CODE BEGIN 4 */

//void bigshoot1_pwm_setvalue(uint16_t value)
//{
//    TIM_OC_InitTypeDef sConfigOC;
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = value;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  
//}
//void bigshoot2_pwm_setvalue(uint16_t value)
//{
//    TIM_OC_InitTypeDef sConfigOC;
//    sConfigOC.OCMode = TIM_OCMODE_PWM1;
//    sConfigOC.Pulse = value;
//    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//    HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);
//    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);  
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
