#ifndef __KINEMATICS_H
#define __KINEMATICS_H
#include "main.h"

typedef struct
{
	char r;
	float * POS_left;
	float * POS_right;
	int T;
	char h;
	char flag;
	char le;
}Leg_data;
void ass_cmd(MOTOR_send *MOTOR,unsigned short id,unsigned short mode,float T,float W,float Pos,float K_P,float K_W);
float x_odd(char r,float t);
float y_odd(char h,float t);
void sport_middle(float x,float y,float *POS_left,float *POS_right );
void sport(char h,char r,float t,float * POS_left,float * POS_right);
extern float POS2,POS1,POS3,POS4,POS5,POS6,POS7,POS8;
extern float zero_1,zero_2,zero_3,zero_4,zero_5,zero_6,zero_7,zero_8;
extern MOTOR_send cmd_1,cmd_2,cmd_3,cmd_4,cmd_5,cmd_6,cmd_7,cmd_8;
void sport_state(char i);
void sport_state_2(char i,int t);
void step(char i,int t);
void run(char i,int t);
void turn_left(char i,int t);
void turn_right(char i,int t);
void jump(char i,int t);
void sport_state_all(char i,int t);
void sport_middle(float x,float y,float *POS_left,float *POS_right );
void stop();
void sport_middle(float x,float y,float *POS_left,float *POS_right );
void jump_qian(char i,int t);


//void sport_all(Leg_data *Leg,float t);

#endif
