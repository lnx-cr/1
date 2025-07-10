#include "main.h"
#include "arm_math.h"
#include "Kinematics.h"
#include "arm_math.h"
#include <math.h>
#include "tim.h"

float x_angle_h,y_angle_h,pitch_z=1.5,Roll_z=0;
float error_i_y,error_i_x;
void stand(float *lq,float * lh,float * rq,float *rh,float *Pitch,float *Roll)
{
	float py=1.5,iy=0.15,id,px=2,ix=0.1;
	*Pitch=*Pitch-pitch_z;
	*Roll=*Roll-Roll_z;
	y_angle_h=-sin(*Roll*3.14f/180)*231.25f;
	x_angle_h=-sin(*Pitch*3.14f/180)*147.5f;

	error_i_y=error_i_y+y_angle_h;

	if(error_i_y<=-400)error_i_y=-400;
	if(error_i_y>=400)error_i_y=400;
	error_i_x=error_i_x+x_angle_h;
	
	if(error_i_x<=-400)error_i_x=-400;
	if(error_i_x>=400)error_i_x=400;
//	x_angle_h=0;
//	y_angle_h=0;
	*lh=-(py*y_angle_h+error_i_y*iy)+px*x_angle_h+error_i_x*ix;
	*rh=(py*y_angle_h+error_i_y*iy)+px*x_angle_h+error_i_x*ix;
	*lq=(py*y_angle_h+error_i_y*iy)+px*x_angle_h+error_i_x*ix;
	*rq=-(py*y_angle_h+error_i_y*iy)+px*x_angle_h+error_i_x*ix;
}
float turn_st(float *Yaw,float *Yaw_goal)
{
	float dx,error,kp=0.1,ki=0.01;
	static float error_i;
	error=*Yaw_goal-*Yaw;
	error_i=error_i+error;
	if(error_i>100)error_i=100;
	if(error_i<-100)error_i=-100;
	dx=error*kp+ki*error_i;
	return dx;
}
float yao_turn(uint16_t ya,char state)
{
	float dx;
	if(state==1)
	{
		dx=(ya-1000)*0.05;
	}
	else if(state==2)
	{
		dx=(ya-1000)*0.05;
	}
	
	
	return dx;
}
void angle_servo(int16_t x,int16_t y)
{
	float angle_yaw,angle_pitch;
	float px=0.25,py=0.2,ix=0,iy=0,dx=0,dy=0;
	float error_x=0,error_y=0,error_dx,error_dy;
	static float error_xi,error_yi,error_x_last,error_y_last;
	
	error_x=x-320;
	error_y=y-240;
	error_xi=error_xi+error_x;
	error_yi=error_yi+error_y;
	if(error_xi<=-400)error_xi=-400;
	if(error_xi>=400)error_xi=400;
	
	if(error_yi<=-400)error_yi=-400;
	if(error_yi>=400)error_yi=400;
	
	angle_yaw=error_x*px+error_x*ix+dx*(error_x-error_x_last);
	angle_pitch=error_y*py+error_y*iy+dy*(error_y-error_y_last);
	error_x_last=error_x;
	error_y_last=error_y;
	
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_1,(angle_yaw/270*2000+1500));
	__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,(angle_pitch/270*2000+1500));
//	vTaskDelay(200);
}
