#include "rtthread.h"
#include "main.h"
#include "stdio.h"

#include "Kinematics.h"
#include "uart_bsp.h"
#include "PID.h"

extern MOTOR_send cmd_1,cmd_2,cmd_3,cmd_4,cmd_5,cmd_6,cmd_7,cmd_8;  
extern MOTOR_recv data_1,data_2,data_3,data_4,data_5,data_6,data_7,data_8;
extern char state,state_fur;
extern uint16_t CH[18];
extern float POS2,POS1,POS3,POS4,POS5,POS6,POS7,POS8;
HAL_StatusTypeDef flag;
char flag_state_turn=1,flag_jump=0;
extern float zero_1,zero_2,zero_3,zero_4,zero_5,zero_6,zero_7,zero_8;
float zero_1_a,zero_2_a,zero_3_a,zero_4_a,zero_5_a,zero_6_a,zero_7_a,zero_8_a;
char i_walk=0,i_run=0,i_sr=0,i_l=0,i_r=0,i_step=0,i_j=0;
int t_walk=0,t_run=0,t_sr=0,t_l=0,t_r=0,t_step=0,t_j=0;
extern float hui_1,hui_2,hui_3,hui_4,hui_5,hui_6,hui_7,hui_8;
extern uint16_t CH_pan[18];
int16_t x,y;

struct rt_thread sportTask_thread;
rt_uint8_t rt_sportTask_thread_stack[512];
void SportTask(void *parameter);
struct rt_thread positionTask_thread;
rt_uint8_t rt_positionTask_thread_stack[512];
void PositionTask(void *parameter);
struct rt_thread HCITask_thread;
rt_uint8_t rt_HCITask_thread_stack[512];
void HCITask03(void *parameter);

void MX_RT_Thread_Init(void)
{
	//初始化LED线程
	rt_thread_init(&sportTask_thread,"sportTask",SportTask,RT_NULL,&rt_sportTask_thread_stack[0],sizeof(rt_sportTask_thread_stack),3,20);
	rt_thread_init(&positionTask_thread,"positionTask",PositionTask,RT_NULL,&rt_positionTask_thread_stack[0],sizeof(rt_positionTask_thread_stack),4,20);
	rt_thread_init(&HCITask_thread,"HCITask",HCITask03,RT_NULL,&rt_HCITask_thread_stack[0],sizeof(rt_HCITask_thread_stack),5,20);
	//开启线程调度
	rt_thread_startup(&sportTask_thread);
	rt_thread_startup(&positionTask_thread);
	rt_thread_startup(&HCITask_thread);
}


void MX_RT_Thread_Process(void)//判断程序是否正常运行
{
	printf("Hello world!!!");
	rt_thread_delay(2000);
}


void SportTask(void *parameter)
{
	while(1)
	{
		flag=SERVO_Send_recv(&cmd_1, &data_1);	
		rt_thread_delay(1);
		SERVO_Send_recv(&cmd_2, &data_2);	
		rt_thread_delay(1);
		SERVO_Send_recv_right(&cmd_3, &data_3);	
		rt_thread_delay(1);
		SERVO_Send_recv_right(&cmd_4, &data_4);	
		rt_thread_delay(1);
		SERVO_Send_recv(&cmd_5, &data_5);	
		rt_thread_delay(1);
		SERVO_Send_recv(&cmd_6, &data_6);	
		rt_thread_delay(1);
		SERVO_Send_recv_right(&cmd_7, &data_7);	
		rt_thread_delay(1);
		SERVO_Send_recv_right(&cmd_8, &data_8);	
		rt_thread_delay(1);
    rt_thread_delay(20);
	}
}

void PositionTask(void *parameter)
{
	rt_thread_delay(1000);
	ass_cmd(&cmd_1,1,1,0,0,zero_1,0.35,0.2);
	ass_cmd(&cmd_2,2,1,0,0,zero_2,0.35,0.2);
	ass_cmd(&cmd_3,3,1,0,0,zero_3,0.35,0.2);
	ass_cmd(&cmd_4,4,1,0,0,zero_4,0.35,0.2);
	ass_cmd(&cmd_5,5,1,0,0,zero_5,0.35,0.2);
	ass_cmd(&cmd_6,6,1,0,0,zero_6,0.35,0.2);
	ass_cmd(&cmd_7,7,1,0,0,zero_7,0.35,0.2);
	ass_cmd(&cmd_8,8,1,0,0,zero_8,0.35,0.2);

	rt_thread_delay(1000);
	zero_1_a=zero_1;
	zero_2_a=zero_2;
	zero_3_a=zero_3;
	zero_4_a=zero_4;
	zero_5_a=zero_5;
	zero_6_a=zero_6;
	zero_7_a=zero_7;
	zero_8_a=zero_8;
	char flag=0;
	while(1)
	{
		if(flag_state_turn)
		{
			dx=yao_turn(CH[3],state);
		}
		switch(state)
		{
			case 0:
				stop();
				break;
			case 1:
				flag_state_turn=0;
				sport_state_all(i_walk,t_walk);
				
				t_walk++;
					if(flag!=1)
					{
						flag=1;
					}
					if(t_walk>=100)
					{
						t_walk=0;
						i_walk++;
						if(i_walk>=4)
						{
							i_walk=0;
							flag=0;
							flag_state_turn=1;
						}
					}
				break;
			case 2:
				flag_state_turn=0;
				run(i_run,t_run);
				t_run++;
				if(flag!=1)
						{
							flag=1;
						}
				if(t_run>=20)
				{
					t_run=0;
					i_run++;
					
					if(i_run>=2)
					{
						i_run=0;
						flag_state_turn=1;
					}
				}
				break;

		case 4:
			flag_state_turn=0;
			turn_left(i_l,t_l);
			t_l++;
			
			if(t_l>=50)
			{
				t_l=0;
				i_l++;
				if(i_l>=2)
				{
					i_l=0;
					flag_state_turn=1;
				}
			}
			break;
		case 5:
			flag_state_turn=0;
			turn_right(i_r,t_r);
			t_r++;
			if(t_r>=50)
			{
				t_r=0;
				i_r++;
				if(i_r>=2)
				{
					i_r=0;
					flag_state_turn=1;
				}
			}
			break;
		case 6:
			flag_state_turn=0;
			step(i_step,t_step);
			t_step++;
			if(t_step>=30)
			{
				t_step=0;
				i_step++;
				if(i_step>=2)
				{
					i_step=0;
					flag_state_turn=1;
				}
			}
			break;
		case 7:
			flag_state_turn=0;
			jump_qian(i_j,t_j);
			i_j++;
			if(i_j>=3)
			{
				i_j=0;
				state_fur=0;
				flag_state_turn=1;
			}
			break;
		case 10:
			ass_cmd(&cmd_1,1,1,0,0,hui_1,0.4,0.2);
			ass_cmd(&cmd_2,2,1,0,0,hui_2,0.4,0.2);
			ass_cmd(&cmd_3,3,1,0,0,hui_3,0.4,0.2);
			ass_cmd(&cmd_4,4,1,0,0,hui_4,0.4,0.2);
			ass_cmd(&cmd_5,5,1,0,0,hui_5,0.4,0.2);
			ass_cmd(&cmd_6,6,1,0,0,hui_6,0.4,0.2);
			ass_cmd(&cmd_7,7,1,0,0,hui_7,0.4,0.2);
			ass_cmd(&cmd_8,8,1,0,0,hui_8,0.4,0.2);
			break;
		default:
			break;
		}
		
		if((state_fur!=state) &&flag_state_turn==1)
		{
			state=state_fur;
		}
    rt_thread_delay(10);
  }
}

void HCITask03(void *parameter)
{
	for(;;)
  {
//		if(CH[0]<1830&&CH[2]<1760 )
//			{
				if(CH[4]==1000 && CH[5]==1000){state_fur=0;flag_jump=1;}
				if(CH[4]==200 && CH[5]==1000)state_fur=1;
				if(CH[4]==1800 && CH[5]==1000)state_fur=2;
				if(CH[4]==1000 && CH[5]==200)state_fur=6;
				if((CH[4]==1000 && CH[5]==1800)&&flag_jump==1){state_fur=7;flag_jump=0;}
				if(CH[4]==1000 && CH[3]>1200)state_fur=4;
				if(CH[4]==1000 && CH[3]<800)state_fur=5;
//			}
//		else if((t==0&&CH[0]==1843)&&CH[2]>1800 )
//		{ 
//			state_fur=10;
//			t=1;
//		}
		stand(&lq,&lh,&rq,&rh,&Pitch,&Roll);

		GET_angles(&Yaw,&Pitch,&Roll);
		if(flag_u1==1)
		{
			GET_xy(&x,&y);
			angle_servo(x,y);
		}
    rt_thread_delay(20);
  }
}

