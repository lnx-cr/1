#include "main.h"
#include "arm_math.h"
#include "Kinematics.h"
#include "PID.h"
#include <rtthread.h>

#include "stdio.h"


#define _USE_MATH_DEFINES

extern float POS2,POS1,POS3,POS4,POS5,POS6,POS7,POS8;
extern float zero_1,zero_2,zero_3,zero_4,zero_5,zero_6,zero_7,zero_8;
extern MOTOR_send cmd_1,cmd_2,cmd_3,cmd_4,cmd_5,cmd_6,cmd_7,cmd_8;  
float kp=0.6,kd=0.04,kp_3=0.4,kp_run=1.1,kd_run=0.05;
float kp_j=0.9,kd_j=0.02,kd_3=0.04;
char re_r=180,re_r_2=90,step_h=30,turn_r,run_h=32;
int a=100,time_step=15,time_run=15,time_turn=50,T_walk=100;
Leg_data l_q,l_h,r_q,r_h;

int x_main=150;


void ass_cmd(MOTOR_send *MOTOR,unsigned short id,unsigned short mode,float T,float W,float Pos,float K_P,float K_W)//给结构体赋值
{
	MOTOR->id=id; 	
	MOTOR->mode=mode;
	MOTOR->T=T;
	MOTOR->W=W;
	MOTOR->Pos=Pos;
	MOTOR->K_P=K_P;
	MOTOR->K_W=K_W;
}
float x_odd(char r,float t)//x的摆线方程,单位均为cm   t/T-1/2PI*sin(2PIt/T)
{
	float x;
	float theta,sin_theta;
	theta=31.41f*t;
	sin_theta=arm_sin_f32(theta);
	x=r*(t/a-sin_theta/6.282f)-(float)r/2;
	return x;
}
float x_odd_2(char r,float t)//x的摆线方程,单位均为cm   t/T-1/2PI*sin(2PIt/T)
{
	float x;
	float theta,sin_theta;
	theta=6.28f*t/time_turn;
	sin_theta=arm_sin_f32(theta);
	x=r*(t/time_turn-sin_theta/6.28f)-(float)r/2;
	x=-x;
	return x;
}
float x_odd_2_run(char r,float t)//x的摆线方程,单位均为cm   t/T-1/2PI*sin(2PIt/T)
{
	float x;
	float theta,sin_theta;
	theta=6.2832f*t/time_run;
	sin_theta=arm_sin_f32(theta);
	x=r*(t/time_run-sin_theta/6.28f)-(float)r/2;
	x=-x;
	return x;
}

float y_odd(char h,float t)//由于公式原因和计算方便，真实高度的最大值为2h，，y的摆线方程
{
	float y;
	float theta,cos_theta;
	theta=31.41f*t;
	cos_theta=arm_cos_f32(theta);
	y=h*(1-cos_theta);
	return y;
}
float y_odd_2(char h,float t)//由于公式原因和计算方便，真实高度的最大值为2h，，y的摆线方程
{
	float y;
	float theta,cos_theta;
	theta=6.2832f*t/time_turn;
	cos_theta=arm_cos_f32(theta);
	y=h*(1-cos_theta);
	return y;
}
float y_odd_run(char h,float t)//由于公式原因和计算方便，真实高度的最大值为2h，，y的摆线方程
{
	float y;
	float theta,cos_theta;
	theta=6.2832f*t/time_run;
	cos_theta=arm_cos_f32(theta);
	y=h*(1-cos_theta);
	return y;
}

void sport_middle(float x,float y,float *POS_left,float *POS_right )//更具x，y得出两只腿的角度，left为短腿的朝前的电机
{
	float theta1,theta2;
	float theta1_middle,theta1_middle2,theta2_middle;
//	switch (s)
//	{
//		case 1:y=y+lq;break;
//		case 2:y=y+lh;break;
//		case 3:y=y+rq;break;
//		case 4:y=y+r;break;
//	}
	theta1_middle=x*x+y*y-381*y;
	theta1_middle2=220*sqrt(x*x+(190.5f-y)*(190.5f-y));
	theta1_middle=theta1_middle/theta1_middle2;
	theta1=acosf (theta1_middle);
	theta2_middle=x/(190.5f-y);
	theta2=atanf(theta2_middle);
	*POS_left=1.57f+theta2-theta1;
	*POS_right=1.57f-theta2-theta1;
	*POS_left=*POS_left*6.28f;
	*POS_right=*POS_right*6.28f;
	
}
void sport(char h,char r,float t,float * POS_left,float * POS_right)
{
	float x,y;
//	r=r+dx;
	x=x_odd(r,t);
	y=y_odd(h,t);
	sport_middle(x,y,POS_left,POS_right);
}
void sport_2(char h,char r,float t,float * POS_left,float * POS_right)
{
	float x,y;
//	r=r+dx;
	x=x_odd_2(r,t);
	y=y_odd_2(h,t);
	sport_middle(x,y,POS_left,POS_right);
}
void sport_2_run(char h,char r,float t,float * POS_left,float * POS_right,char leg)
{
	float x,y;
//	r=r+dx;
	switch (leg)
	{
		case 1:r=r-dx;break;
		case 2:r=r-dx;break;
		case 3:r=r+dx;break;
		case 4:r=r+dx;break;
		
	}
	x=x_odd_2_run(r,t);
	y=y_odd_run(h,t);
	sport_middle(x,y,POS_left,POS_right);
}
void sport_ping(char r,float t,float * POS_left,float * POS_right)
{
	float x;
	int T;
//	r=r+dx;
	T=3*a;
	x=-r+2*r*t/T;
	sport_middle(x,0,POS_left,POS_right);
}
void sport_ping_run(char r,float t,float * POS_left,float * POS_right,char leg)
{
	float x;
//	r=r+dx;
	switch (leg)
	{
		case 1:r=r-dx;break;
		case 2:r=r-dx;break;
		case 3:r=r+dx;break;
		case 4:r=r+dx;break;
		
	}
	x=-r+2*r*t/time_run;
	sport_middle(x,0,POS_left,POS_right);
}
void step_y(char h,int t,float * POS_left,float * POS_right)
{
	float y;
	if(t<time_step)
	{
		y=h*t/time_step;
	}
	else
	{
		y=h-h*t/time_step;
	}
	sport_middle(0,y,POS_left,POS_right);
}

//void sport_all(Leg_data *Leg,float t)//left为腿在平衡时向前（正时为从后往前,flag判断从后往前（1）还是从前往后）
//{
//	float x,y;
//	char s;
//	s=Leg->le;
//	float theta,sin_theta,cos_theta;
//	theta=6.2832f *t/Leg->T;
//	sin_theta=arm_sin_f32(theta);
//	switch (s)
//	{
//		case 1:Leg->r=Leg->r+dx;break;
//		case 2:Leg->r=Leg->r+dx;break;
//		case 3:Leg->r=Leg->r-dx;break;
//		case 4:Leg->r=Leg->r-dx;break;
//		
//	}
//	
//	x=Leg->r*(t/a-sin_theta/6.2832f )-(float)Leg->r/2;
//	if(Leg->flag==1)
//		{
//			x=-x;
//		}
//	cos_theta=arm_cos_f32(theta);
//	y=Leg->h*(1-cos_theta);
//	sport_middle(x,y,Leg->POS_left,Leg->POS_right);
//}
char f;
void sport_all(char h,char r,float t,float * POS_left,float * POS_right,int T,char flag,char leg)//left为腿在平衡时向前（正时为从后往前,flag判断从后往前（1）还是从前往后）
{
	float x,y;
	float theta,sin_theta,cos_theta;
	theta=6.2832f *t/T;
	sin_theta=arm_sin_f32(theta);
	switch (leg)
	{
		case 1:r=r-dx;f=r;break;
		case 2:r=r-dx;break;
		case 3:r=r+dx;break;
		case 4:r=r+dx;break;
		
	}
//	r=r+dx;
	x=r*(t/T-sin_theta/6.2832f )-(float)r/2;
	if(flag==1)
		{
			x=-x;
		}
	cos_theta=arm_cos_f32(theta);
	y=h*(1-cos_theta);
	sport_middle(x,y,POS_left,POS_right);
}
void ping_all(char r,float t,float * POS_left,float * POS_right,int T,char flag,char leg)//默认从前往后,falg为1则是从后往前
{
	float x;
	int T_all;
	T_all=3*T;
//	r=r+dx;
	switch (leg)
	{
		case 1:r=r-dx;f=r;break;
		case 2:r=r-dx;break;
		case 3:r=r+dx;break;
		case 4:r=r+dx;break;
		
	}
	x=-r+2*r*t/T_all;
	if(flag==0)
	{
		x=-x;
	}
	sport_middle(x,0,POS_left,POS_right);
}
void stop()
{

	ass_cmd(&cmd_1,1,1,0,0,zero_1,0.5,0.2);
	ass_cmd(&cmd_2,2,1,0,0,zero_2,0.5,0.2);
	ass_cmd(&cmd_3,3,1,0,0,zero_3,0.5,0.2);
	ass_cmd(&cmd_4,4,1,0,0,zero_4,0.5,0.2);
	ass_cmd(&cmd_5,5,1,0,0,zero_5,0.5,0.2);
	ass_cmd(&cmd_6,6,1,0,0,zero_6,0.5,0.2);
	ass_cmd(&cmd_7,7,1,0,0,zero_7,0.5,0.2);
	ass_cmd(&cmd_8,8,1,0,0,zero_8,0.5,0.2);
}


void sport_state_2(char i,int t)
{
	int t1,t2,t3,t4;
	switch (i)
	{
		case 0:
			t4=2*a+t;
			t2=t;
			t3=a+t;
			sport_ping(re_r_2,t2,&POS4,&POS3);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
		
			sport_ping(re_r_2,t3,&POS5,&POS6);
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
		
			sport_ping(re_r_2,t4,&POS7,&POS8);
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);

		
			sport_2(32,re_r,t,&POS2,&POS1);
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);

			break;
		case 1:
			t1=t;
			t2=a+t;
			t3=2*a+t;
			sport_ping(re_r_2,t2,&POS4,&POS3);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
		
			sport_ping(re_r_2,t3,&POS5,&POS6);
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
		
			sport_2(32,re_r,t,&POS7,&POS8);
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);

		
			sport_ping(re_r_2,t1,&POS2,&POS1);
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);
			break;
		case 2:
			
			t1=t+a;
			t4=t;
			t2=t+2*a;
			sport_ping(re_r_2,t2,&POS4,&POS3);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
		
			sport_2(32,re_r,t,&POS5,&POS6);
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
		
			sport_ping(re_r_2,t4,&POS7,&POS8);
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);

		
			sport_ping(re_r_2,t1,&POS2,&POS1);
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);
			break;
			case 3:
				t1=t+2*a;
				t4=t+a;
				t3=t;
				sport_2(32,re_r,t,&POS4,&POS3);
				POS3=POS3+zero_3;
				POS4=POS4+zero_4;
				ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
				ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
			
				sport_ping(re_r_2,t3,&POS5,&POS6);
				POS5=-POS5+zero_5;
				POS6=-POS6+zero_6;
				ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
				ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
			
				sport_ping(re_r_2,t4,&POS7,&POS8);
				POS7=-POS7+zero_7;
				POS8=-POS8+zero_8;
				ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
				ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);

			
				sport_ping(re_r_2,t1,&POS2,&POS1);
				POS1=POS1+zero_1;
				POS2=POS2+zero_2;
				ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
				ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);
				break;
			
	}
}
void sport_state_all(char i,int t)
{
	int t1,t2,t3,t4;
	switch (i)
	{
		case 0:
			t4=2*a+t;
			t2=t;
			t3=a+t;
			ping_all(re_r_2,t2,&POS4,&POS3,T_walk,1,2);
			ping_all(re_r_2,t3,&POS5,&POS6,T_walk,1,3);
			ping_all(re_r_2,t4,&POS7,&POS8,T_walk,1,4);
			sport_all(32,re_r,t,&POS2,&POS1,T_walk,1,1);

			break;
		case 1:
			t1=t;
			t2=a+t;
			t3=2*a+t;
			ping_all(re_r_2,t2,&POS4,&POS3,T_walk,1,2);
			ping_all(re_r_2,t3,&POS5,&POS6,T_walk,1,3);
			sport_all(32,re_r,t,&POS7,&POS8,T_walk,1,4);
			ping_all(re_r_2,t1,&POS2,&POS1,T_walk,1,1);

			break;
		case 2:
			
			t1=t+a;
			t4=t;
			t2=t+2*a;
			ping_all(re_r_2,t2,&POS4,&POS3,T_walk,1,2);
			sport_all(32,re_r,t,&POS5,&POS6,T_walk,1,3);
			ping_all(re_r_2,t4,&POS7,&POS8,T_walk,1,4);
			ping_all(re_r_2,t1,&POS2,&POS1,T_walk,1,1);

			break;
			case 3:
				t1=t+2*a;
				t4=t+a;
				t3=t;
				sport_all(32,re_r,t,&POS4,&POS3,T_walk,1,2);
				ping_all(re_r_2,t3,&POS5,&POS6,T_walk,1,3);
				ping_all(re_r_2,t4,&POS7,&POS8,T_walk,1,4);
				ping_all(re_r_2,t1,&POS2,&POS1,T_walk,1,1);
				

				break;
			
	}
	POS1=POS1+zero_1;
	POS2=POS2+zero_2;
	POS3=POS3+zero_3;
	POS4=POS4+zero_4;
	POS5=-POS5+zero_5;
	POS6=-POS6+zero_6;
	POS7=-POS7+zero_7;
	POS8=-POS8+zero_8;
	ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
	ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);
	ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
	ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
	ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
	ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
	ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
	ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);
}
void step(char i,int t)
{
	switch (i)
	{
		case 0:
			step_y(step_h,t,&POS7,&POS8);
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
		
			ass_cmd(&cmd_3,3,1,0,0,zero_3,0.35,0.12);
			ass_cmd(&cmd_4,4,1,0,0,zero_4,0.35,0.12);
			ass_cmd(&cmd_5,5,1,0,0,zero_5,0.35,0.12);
			ass_cmd(&cmd_6,6,1,0,0,zero_6,0.35,0.12);
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp,0.02);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp,0.02);
		
			step_y(step_h,t,&POS2,&POS1);
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp,0.02);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp,0.02);

			break;
		case 1:
			step_y(step_h,t,&POS4,&POS3);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
		
			step_y(step_h,t,&POS5,&POS6);
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
		
			ass_cmd(&cmd_1,1,1,0,0,zero_1,0.35,0.12);
			ass_cmd(&cmd_2,2,1,0,0,zero_2,0.35,0.12);
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp,0.02);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp,0.02);
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp,0.02);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp,0.02);
			ass_cmd(&cmd_7,7,1,0,0,zero_7,0.35,0.12);
			ass_cmd(&cmd_8,8,1,0,0,zero_8,0.35,0.12);
			break;

			
			
	}
}

void run(char i,int t)
{
	switch (i)
	{
		case 0:
			sport_ping_run(re_r_2,t,&POS5,&POS6,3);
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
			sport_ping_run(re_r_2,t,&POS4,&POS3,2);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp_run,kd_run);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp_run,kd_run);
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp_run,kd_run);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp_run,kd_run);

		
			sport_2_run(run_h,re_r,t,&POS2,&POS1,1);
			sport_2_run(run_h,re_r,t,&POS7,&POS8,4);
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp_run,kd_run);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp_run,kd_run);
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp_run,kd_run);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp_run,kd_run);

			break;
		case 1:
			sport_2_run(run_h,re_r,t,&POS4,&POS3,2);
			sport_2_run(run_h,re_r,t,&POS5,&POS6,3);
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp_run,kd_run);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp_run,kd_run);
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp_run,kd_run);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp_run,kd_run);
			

		
			sport_ping_run(re_r_2,t,&POS2,&POS1,1);
			sport_ping_run(re_r_2,t,&POS7,&POS8,4);
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp_run,kd_run);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp_run,kd_run);
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp_run,kd_run);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp_run,kd_run);
			break;
	}
}

void sport_ping_turn(char r,float t,float * POS_left,float * POS_right)
{
	float x;
	x=-r+2*r*t/time_turn;
	x=-x;
	sport_middle(x,0,POS_left,POS_right);
}
void turn_left(char i,int t)
{
	switch (i)
	{
		case 0:
		
			sport_2(32,60,t,&POS7,&POS8);
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);

		
			sport_2(32,60,t,&POS1,&POS2);
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);
			
		
			sport_ping_turn(30,t,&POS6,&POS5);
			sport_ping_turn(30,t,&POS4,&POS3);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
		
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
			break;
		case 1:
			sport_ping_turn(30,t,&POS8,&POS7);
			sport_ping_turn(30,t,&POS2,&POS1);
			
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);
		
			sport_2(32,60,t,&POS3,&POS4);
			sport_2(32,60,t,&POS5,&POS6);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
		
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
			break;
	}
}
void turn_right(char i,int t)
{
	switch (i)
	{
		case 0:
		
			sport_2(32,60,t,&POS8,&POS7);
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);

		
			sport_2(32,60,t,&POS2,&POS1);
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);
			
		
			sport_ping_turn(30,t,&POS5,&POS6);
			sport_ping_turn(30,t,&POS3,&POS4);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
		
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
			break;
		case 1:
			sport_ping_turn(30,t,&POS7,&POS8);
			sport_ping_turn(30,t,&POS1,&POS2);
			
			POS1=POS1+zero_1;
			POS2=POS2+zero_2;
			POS7=-POS7+zero_7;
			POS8=-POS8+zero_8;
			ass_cmd(&cmd_1,1,1,0,0,POS1,kp,kd);
			ass_cmd(&cmd_2,2,1,0,0,POS2,kp,kd);
			ass_cmd(&cmd_7,7,1,0,0,POS7,kp,kd);
			ass_cmd(&cmd_8,8,1,0,0,POS8,kp,kd);
		
			sport_2(32,60,t,&POS4,&POS3);
			sport_2(32,60,t,&POS6,&POS5);
			POS3=POS3+zero_3;
			POS4=POS4+zero_4;
			POS5=-POS5+zero_5;
			POS6=-POS6+zero_6;
		
			ass_cmd(&cmd_3,3,1,0,0,POS3,kp,kd);
			ass_cmd(&cmd_4,4,1,0,0,POS4,kp,kd);
			ass_cmd(&cmd_5,5,1,0,0,POS5,kp,kd);
			ass_cmd(&cmd_6,6,1,0,0,POS6,kp,kd);
			break;
	}
}
void jump(char i,int t)
{
	switch (i)
	{
		case 0:
			sport_middle(0,40,&POS7,&POS8);
			break;
		case 1:
			sport_middle(0,-40,&POS7,&POS8);break;
		case 2:
			sport_middle(0,30,&POS7,&POS8);
			
			break;
		}
	POS1=POS7+zero_1;
	POS2=POS8+zero_2;
	POS3=POS7+zero_3;
	POS4=POS8+zero_4;
	POS5=-POS7+zero_5;
	POS6=-POS8+zero_6;
	POS7=-POS7+zero_7;
	POS8=-POS8+zero_8;

	
	ass_cmd(&cmd_1,1,1,0,0,POS1,kp_j,0.02);
	ass_cmd(&cmd_2,2,1,0,0,POS2,kp_j,0.02);

	ass_cmd(&cmd_3,3,1,0,0,POS3,kp_j,0.02);
	ass_cmd(&cmd_4,4,1,0,0,POS4,kp_j,0.02);
	ass_cmd(&cmd_5,5,1,0,0,POS5,kp_j,0.02);
	ass_cmd(&cmd_6,6,1,0,0,POS6,kp_j,0.02);
	ass_cmd(&cmd_7,7,1,0,0,POS7,kp_j,0.02);
	ass_cmd(&cmd_8,8,1,0,0,POS8,kp_j,0.02);
	switch (i)
	{
		case 0:rt_thread_delay(700);break;
		case 1:rt_thread_delay(200);break;
		case 2:rt_thread_delay(300);break;
	}
		
	
}

void jump_qian(char i,int t)
{
	switch (i)
	{
		case 0:
			sport_middle(-70,40,&POS7,&POS8);
			break;
		case 1:
			sport_middle(70,-30,&POS7,&POS8);break;
		case 2:
			sport_middle(-70,30,&POS7,&POS8);
			
			break;
		}
	POS1=POS8+zero_1;
	POS2=POS7+zero_2;
	POS3=POS8+zero_3;
	POS4=POS7+zero_4;
	POS5=-POS7+zero_5;
	POS6=-POS8+zero_6;
	POS7=-POS7+zero_7;
	POS8=-POS8+zero_8;

	
	ass_cmd(&cmd_1,1,1,0,0,POS1,kp_j,0.02);
	ass_cmd(&cmd_2,2,1,0,0,POS2,kp_j,0.02);

	ass_cmd(&cmd_3,3,1,0,0,POS3,kp_j,0.02);
	ass_cmd(&cmd_4,4,1,0,0,POS4,kp_j,0.02);
	ass_cmd(&cmd_5,5,1,0,0,POS5,kp_j,0.02);
	ass_cmd(&cmd_6,6,1,0,0,POS6,kp_j,0.02);
	ass_cmd(&cmd_7,7,1,0,0,POS7,kp_j,0.02);
	ass_cmd(&cmd_8,8,1,0,0,POS8,kp_j,0.02);
	switch (i)
	{
		case 0:rt_thread_delay(700);break;
		case 1:rt_thread_delay(300);break;
		case 2:rt_thread_delay(300);break;
	}
		
	
}

