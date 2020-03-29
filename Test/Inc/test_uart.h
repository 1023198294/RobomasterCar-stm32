/**
  *@file test_uart.c
  *@date 2016-12-12
  *@author Albert.D
  *@brief 
  */
  
#ifndef _TEST__UART_H
#define _TEST__UART_H

#include "stm32f4xx_HAL.h"
#define USART_REC_LEN 100

extern uint8_t uart3_rx_buff[1];
extern uint8_t uart6_rx_buff[50];
extern uint8_t uart1_rx_buff[50];
extern uint8_t uart2_rx_buff[10];
extern uint8_t USART_RX_BUF[USART_REC_LEN];
extern uint16_t USART_RX_STA;
extern unsigned int cx,cy;
extern int cx_cy_sum;
extern int height;
extern int text;
#endif

