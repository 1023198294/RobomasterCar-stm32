/**
  *@file test_uart.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */

#include "test_uart.h"
#include "usart.h"
#include "stdio.h"
#include "math.h"
uint8_t USART_RX_BUF[USART_REC_LEN];
uint8_t uart3_rx_buff[1];
uint8_t uart6_rx_buff[50];
uint8_t uart1_rx_buff[50];
uint8_t uart2_rx_buff[10];

uint8_t uart6_tx_buff[21];
uint8_t mybuff[30] = {0};
uint8_t mybuff2[30] = {0};
extern int Yaw_init;
extern int ecode1,ecode2,ecode3,ecode4;
uint8_t aTxMessage[100] = {0};
float Read_Pitch = 0 , Read_GYaw = 0 , 	Read_Roll = 0  ;
float Read_wx = 0 , Read_wy = 0 , 	Read_wz = 0  ;
uint8_t new_sum = 0 ;
uint16_t USART_RX_STA = 0; unsigned int cx = 160,cy = 175;int height = 0;int cx_cy_sum = 0;int text = 0;
int a = 0 , b = 0 , c = 0 , d = 0 ;
int sign1,sign2,sign3,sign4;
int curstate = 0;
int rnd = 0;
//int bg = 0,xa = 0,xb = 0,xc = 0,xd = 0,ed = 0,ptr=0;
//it will be auto callback when usart receive msg completely
extern int xa,xb,xc,xd;
int Q_DATA_TYPE;
void Send_CarBus(uint8_t cmd,float fdata1,float fdata2,float fdata3,float fdata4,int rnd){
	int data1 = (int) fdata1;
	int data2 = (int) fdata2;
	int data3 = (int) fdata3;
	int data4 = (int) fdata4;
	int signal1 = (fdata1>=0)?0:1;
	int signal2 = (fdata2>=0)?0:1;
	int signal3 = (fdata3>=0)?0:1;
	int signal4 = (fdata4>=0)?0:1;
	data1 = abs(data1);
	data2 = abs(data2);
	data3 = abs(data3);
	data4 = abs(data4);
	memset(uart6_tx_buff,0,sizeof(uart6_tx_buff));
	uart6_tx_buff[0] = 0x51;
	uart6_tx_buff[20] = 0x52;
	uart6_tx_buff[1] = cmd;
	uart6_tx_buff[2] = rnd/10+'0';
	uart6_tx_buff[3] = rnd%10+'0';
	uart6_tx_buff[4] = signal1;
	uart6_tx_buff[5] = data1/100+'0';
	uart6_tx_buff[6] = data1/10%10+'0';
	uart6_tx_buff[7] = data1%10+'0';
	uart6_tx_buff[8] = signal2;
	uart6_tx_buff[9] = data2/100+'0';
	uart6_tx_buff[10] = data2/10%10+'0';
	uart6_tx_buff[11] = data2%10+'0';
	uart6_tx_buff[12] = signal3;
	uart6_tx_buff[13] = data3/100+'0';
	uart6_tx_buff[14] = data3/10%10+'0';
	uart6_tx_buff[15] = data3%10+'0';
	uart6_tx_buff[16] = signal4;
	uart6_tx_buff[17] = data4/100+'0';
	uart6_tx_buff[18] = data4/10%10+'0';
	uart6_tx_buff[19] = data4%10+'0';
	HAL_UART_Transmit(&huart6,uart6_tx_buff,sizeof(uart6_tx_buff),100);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t uart3_date;//half_falg=0,x=0;
	//uint32_t tmp_flag = 0;
  if(huart == &huart6)
  {
		if (uart6_rx_buff[0]==0x51){//如果是命令头，那就长度初始为0
			d = 0;
		}
		mybuff2[d++] = uart6_rx_buff[0];
		if (d==21){
			if (mybuff2[0] == 0x51 && mybuff2[20]==0x52){//发送命令合法
				int new_rnd = (mybuff2[2]-'0')*10+mybuff2[3]-'0';
				if (mybuff2[1] == 0x01){
					rnd = new_rnd;
				}
				if (new_rnd < rnd && rnd != 99){
					d = 0;
					//报错
				}else{
				rnd = (rnd + 1)%100; 
				switch(mybuff2[1]){
					case 0x01://请求确认连接,返回OK帧
							//void Send_CarBus(uint8_t cmd,int data1,int data2,int data3,int data4,int rnd){
								Send_CarBus(0x02,0,0,0,0,rnd);
						break;
					case 0x03:
						Q_DATA_TYPE = mybuff2[7]-'0';
						if (Q_DATA_TYPE==1)
							Send_CarBus(0x04,xa,xb,xc,xd,rnd);
						break;
					case 0x05:
						sign1 = (mybuff2[4]==0) ? 1: -1;
						sign2 = (mybuff2[8]==0) ? 1: -1;
						sign3 = (mybuff2[12]==0) ? 1: -1;
						sign4 = (mybuff2[16]==0) ? 1: -1;
						xa = sign1*((mybuff2[5]-'0')*100+(mybuff2[6]-'0')*10+(mybuff2[7]-'0'));
						xb = sign2*((mybuff2[9]-'0')*100+(mybuff2[10]-'0')*10+(mybuff2[11]-'0'));
						xc = sign3*((mybuff2[13]-'0')*100+(mybuff2[14]-'0')*10+(mybuff2[15]-'0'));
						xd = sign4*((mybuff2[17]-'0')*100+(mybuff2[18]-'0')*10+(mybuff2[19]-'0'));
						Send_CarBus(0x06,0,0,0,0,rnd);
						break;
					case 0x07:
						break;
					default:
						d = 0;
				}
			}
			}else{//重新发送,长度归零
				d = 0;
			}
		}
		__HAL_UART_CLEAR_PEFLAG(&huart6);
		HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);
	}
  if(huart == &huart2)
  {
	  //HAL_UART_Transmit(&huart6,uart2_rx_buff,1,0xffff);
	  if(c == 1){
		d++ ;
		mybuff[d] = uart2_rx_buff[0];
		  //读22位再输出
		  if(d == 21){d = 0 ;c = 0 ;
				//if(mybuff[21]==(mybuff[11]+mybuff[12]+mybuff[13]+mybuff[14]+mybuff[15]+mybuff[16]+mybuff[17]+mybuff[18]+mybuff[19]+mybuff[20]))
				
				Read_wx = ((float)((((short)mybuff[3])<<8)|mybuff[2]) / 32768) * 2000;
		        Read_wy = ((float)((((short)mybuff[5])<<8)|mybuff[4]) / 32768) * 2000;
			    Read_wz = ((float)((((short)mybuff[7])<<8)|mybuff[6]) / 32768) * 2000;
				Read_Roll = ((float)((((short)mybuff[14])<<8)|mybuff[13]) / 32768) * 180;
			    Read_Pitch = ((float)((((short)mybuff[16])<<8)|mybuff[15]) / 32768) * 180;
			  if(Read_Pitch>=180){
				Read_Pitch = Read_Pitch - 360 ;
			  }
			    Read_GYaw = ((float)((((short)mybuff[18])<<8)|mybuff[17]) / 32768) * 180;
			    //printf("Read_wx:%d Read_wy:%d Read_wz:%d \nGyaw:%d   Pitch:%d  Roll:%d\n" ,Read_wx , Read_wy , Read_wz ,  Read_GYaw , Read_Pitch , 	Read_Roll);
				//HAL_UART_Transmit(&huart6,mybuff,22,0xffff);
			    Yaw_init = 2;//此句代码针对z轴不能上电归零的陀螺仪，标记第一次接收到了陀螺仪传回的数据
			  }
	  }
	  if((uart2_rx_buff[0] == 0x52)&&(a == 1) &&(c != 1)){mybuff[1] = 0x52 ; b = 1;d = 1;}else{b = 0 ;}
	  if(a == 1 && b == 1){c = 1;}
	  if((uart2_rx_buff[0] == 0x55) &&(c != 1)){mybuff[0] = 0x55 ; a = 1;d = 0 ;}else{a = 0 ;}
      HAL_UART_Receive_IT(&huart2, uart2_rx_buff, 1);
  }
  if(huart == &huart3){
	
			//__HAL_UART_CLEAR_IDLEFLAG(&huart3);
			//uart3_date = huart3.Instance->SR;  
			//uart3_date = huart3.Instance->DR; 
		//uart3_date = uart3_rx_buff[0];
		uart3_date = *(--huart3.pRxBuffPtr);
		//text++;
		if(!(USART_RX_STA&0x8000))//如果未接收完成
	{
		
			if(uart3_date=='D')
			{
				
				USART_RX_STA|=0x8000;//接收完成
				USART_RX_BUF[USART_RX_STA&0x3fff]=uart3_date;
				
				//if(cx<2||cy<2||cx>(160+158)||cy>(120+118))
					//cx=160,cy=120;
			}
		
			else
				{
				USART_RX_BUF[USART_RX_STA&0x3fff]=uart3_date;
			  USART_RX_STA++;
				if(USART_RX_STA>USART_REC_LEN - 1)USART_RX_STA=0;//接收错误
				}
		}
		
    __HAL_UART_CLEAR_PEFLAG(&huart3);
    
 
	}
}

