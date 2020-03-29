/**
  *@file Yaw_Contral.h
  *@date 2017-5-10
  *@author Jzhiyshen
  *@brief 
  */
#ifndef __Yaw_Contral_H
#define __Yaw_Contral_H
#include <stdint.h >
int16_t Yaw_Chassis(int16_t read_ecoder ,int16_t except_ecoder ,  int16_t speed);
int16_t Yaw_head_Location(float read_imu ,float except_imu ,  float except_v);
int16_t Yaw_head_Speed(float read_imu ,float except_v ,  int except_i);
#endif 
