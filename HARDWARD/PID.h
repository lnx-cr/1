#ifndef __PID_H
#define __PID_H
#include "main.h"
extern MOTOR_recv data_1,data_2,data_3,data_4,data_5,data_6,data_7,data_8;
void stand(float *lq,float * lh,float * rq,float *rh,float *Pitch,float *Roll);
float turn_st(float *Yaw,float *Yaw_goal);
extern float error_i_y,error_i_x;
float yao_turn(uint16_t ya,char state);
void angle_servo(int16_t x,int16_t y);
#endif
