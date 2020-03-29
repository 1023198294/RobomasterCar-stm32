/**
  *@file test_can.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */

#include "test_can.h"
#include "can.h"
#include <stdio.h>
	
	
uint8_t can1_rx_data[8];
uint8_t can2_rx_data[8];
//extern uint8_t can1_rx_data_buf[5][4];

//can filter must be initialized before use
void CanFilter_Init(CAN_HandleTypeDef* hcan)
{
  CAN_FilterConfTypeDef canfilter;
  
  //create memory to save the message, if not will raise error
  static CanTxMsgTypeDef  Tx1Message;
  static CanRxMsgTypeDef  Rx1Message;
  static CanTxMsgTypeDef  Tx2Message;
  static CanRxMsgTypeDef  Rx2Message;
  
  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
  
  //filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;
  
  canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;
  canfilter.FilterActivation = ENABLE;
  canfilter.BankNumber = 14;
  
  //use different filter for can1&can2
  if(hcan == &hcan1)
  {
    canfilter.FilterNumber = 0;
    hcan->pTxMsg = &Tx1Message;
    hcan->pRxMsg = &Rx1Message;
  }
  if(hcan == &hcan2)
  {
    canfilter.FilterNumber = 14;
    hcan->pTxMsg = &Tx2Message;
    hcan->pRxMsg = &Rx2Message;
  }
  
  HAL_CAN_ConfigFilter(hcan, &canfilter);
  
}

//it will be auto callback when can receive msg completely
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
//  switch(hcan->pRxMsg->StdId)
//  {
//    case TEST_CAN1_ID:
//    {
//      can1_rx_data[0] = hcan->pRxMsg->Data[0];
//			can1_rx_data[1] = hcan->pRxMsg->Data[1];
//			can1_rx_data[2] = hcan->pRxMsg->Data[2];
//			can1_rx_data[3] = hcan->pRxMsg->Data[3];
//			can1_rx_data[4] = hcan->pRxMsg->Data[4];
//			can1_rx_data[5] = hcan->pRxMsg->Data[5];
//			can1_rx_data[6] = hcan->pRxMsg->Data[6];
//			can1_rx_data[7] = hcan->pRxMsg->Data[7];
//    }break;
//    case TEST_CAN2_ID:
//    {
//      can2_rx_data[0] = hcan->pRxMsg->Data[0];
//    }break;
  //}
	
  
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
  HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);

}

//CAN send message test
void CAN_Send_Msg(CAN_HandleTypeDef* hcan,  int16_t cm1_iq, int16_t cm2_iq , int16_t cm3_iq, int16_t cm4_iq, uint32_t id, uint8_t len)
{
  
  hcan->pTxMsg->StdId = id;
  hcan->pTxMsg->IDE = CAN_ID_STD;
  hcan->pTxMsg->RTR = CAN_RTR_DATA;
  hcan->pTxMsg->DLC = len;
  
    hcan->pTxMsg->Data[0] = (uint8_t)(cm1_iq >> 8);
    hcan->pTxMsg->Data[1] = (uint8_t)cm1_iq;
    hcan->pTxMsg->Data[2] = (uint8_t)(cm2_iq >> 8);
    hcan->pTxMsg->Data[3] = (uint8_t)cm2_iq;
    hcan->pTxMsg->Data[4] = (uint8_t)(cm3_iq >> 8);
    hcan->pTxMsg->Data[5] = (uint8_t)cm3_iq;
    hcan->pTxMsg->Data[6] = (uint8_t)(cm4_iq >> 8);
    hcan->pTxMsg->Data[7] = (uint8_t)cm4_iq;
  
  HAL_CAN_Transmit(hcan, 1);
}


void print()
{
	for(int i = 0; i < 5; i++)
	{
		printf("ID: 0x20%d\n", i+1);
		for(int j = 0; j < 4; j++)
		{
			printf("%d ", can1_rx_data_buf[i][j]);
		}
		printf("\n");
	}
	printf("\n");
	
}

void Message_buffer(uint8_t buf[][6])
{
	if(0x201 == hcan1.pRxMsg->StdId)
	{
		for(int i = 0; i < 4; i++)
			buf[0][i] = hcan1.pRxMsg->Data[i];
	}
	else if(0x202 == hcan1.pRxMsg->StdId)
	{
		for(int i = 0; i < 4; i++)
			buf[1][i] = hcan1.pRxMsg->Data[i];
	}
	else if(0x203 == hcan1.pRxMsg->StdId)
	{
		for(int i = 0; i < 4; i++)
			buf[2][i] = hcan1.pRxMsg->Data[i];
	}
	else if(0x204 == hcan1.pRxMsg->StdId)
	{
		for(int i = 0; i < 4; i++)
			buf[3][i] = hcan1.pRxMsg->Data[i];
	}
	else if(0x205 == hcan1.pRxMsg->StdId)
	{
		for(int i = 0; i < 6; i++)
			buf[4][i] = hcan1.pRxMsg->Data[i];
	}
	else if(0x206 == hcan1.pRxMsg->StdId)
	{
		for(int i = 0; i < 6; i++)
			buf[5][i] = hcan1.pRxMsg->Data[i];
	}
	else if(0x207 == hcan1.pRxMsg->StdId)
	{
		for(int i = 0; i < 4; i++)
			buf[6][i] = hcan1.pRxMsg->Data[i];
	}
	else if(0x208 == hcan1.pRxMsg->StdId)
	{
		for(int i = 0; i < 6; i++)
			buf[7][i] = hcan1.pRxMsg->Data[i];
	}
}
