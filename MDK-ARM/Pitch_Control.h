/**
  *@file Pitch_Contral.h
  *@date 2017-5-15
  *@author Jzhiyshen
  *@brief 
  */
#ifndef __Pitch_Contral_H
#define __Pitch_Contral_H
#include <stdint.h >
int16_t Pitch_Location(float read_imu ,float except_imu ,  float except_v);
int16_t Pitch_Speed(float read_imu ,float except_v ,  int except_i);
int16_t Pitch_Test(float read_imu ,float except_imu ,  int except_i);
float KalmanFitter5(float MeasVar);
float KalmanFitter7(float MeasVar);
#endif 
