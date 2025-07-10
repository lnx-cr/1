#include "uart_bsp.h"
#include "string.h"
#include "usart.h"
#include "main.h"

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
uint8_t rx_buff[BUFF_SIZE],flag_u1=0;
remoter_t remoter;
RC_Ctl_t  RC_Ctl;
uint16_t CH[18];
uint16_t CH_pan[18]={0};
uint16_t CH_HL[7][7];
uint16_t CH_R[7];
int read;
uint8_t rx_jy[40],rx_middle[6];

union RXData
{
	int16_t Data;
	uint8_t DataHEX[2];
}RXDataYaw,RXDataPitch,RXDataRoll,middle_x,middle_y;

void GET_angles(float *Yaw,float *Pitch,float *Roll)
{
	*Yaw=((float)RXDataYaw.Data*180/32768);
	*Pitch=((float)RXDataPitch.Data*180/32768);
	*Roll=((float)RXDataRoll.Data*180/32768);
	
}
void GET_xy(int16_t *x,int16_t *y)
{
	middle_x.DataHEX[0]=rx_middle[2];
	middle_x.DataHEX[1]=rx_middle[1];
	middle_y.DataHEX[0]=rx_middle[4];
	middle_y.DataHEX[1]=rx_middle[3];
	*x=middle_x.Data;
	*y=middle_y.Data;
}
char flag_fr;
char pan();
void sbus_frame_parse(remoter_t *remoter, uint8_t *buf)
{
//	char flag;
//	flag=	pan();
//	if(flag!=1)
//	{
//		CH[ 0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;
//		CH[ 1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;
//		CH[ 2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 )) & 0x07FF;
//		CH[ 3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;
//		CH[ 4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;
//		CH[ 5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 )  | (int16_t)buf[10] <<  9 ) & 0x07FF;
//		CH[ 6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;
//		CH[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
		CH[ 0] = ((int16_t)buf[ 1] >> 0 | ((int16_t)buf[ 2] << 8 )) & 0x07FF;
		CH[ 1] = ((int16_t)buf[ 2] >> 3 | ((int16_t)buf[ 3] << 5 )) & 0x07FF;
		CH[ 2] = ((int16_t)buf[ 3] >> 6 | ((int16_t)buf[ 4] << 2 )| (int16_t)buf[5] <<  10) & 0x07FF;
		CH[ 3] = ((int16_t)buf[ 5] >> 1 | ((int16_t)buf[ 6] << 7 )) & 0x07FF;
		CH[ 4] = ((int16_t)buf[ 6] >> 4 | ((int16_t)buf[ 7] << 4 )) & 0x07FF;
		CH[ 5] = ((int16_t)buf[ 7] >> 7 | ((int16_t)buf[ 8] << 1 )  | (int16_t)buf[9] <<  9 ) & 0x07FF;
		CH[ 6] = ((int16_t)buf[9] >> 2 | ((int16_t)buf[10] << 6 )) & 0x07FF;
		CH[ 7] = ((((int16_t)buf[10] >> 5)&0x0007) | ((int16_t)buf[11] << 3 )) & 0x07FF;
		
//	}
	
}
char pan(void)
{ 
	char i=0,j=0;
	char flag=0;
	for(i=0;i<12;i++)
	{
			if(rx_buff[i]==0x0000)
			{
				flag=1;
				break;
			}
			
	}
	return flag;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == UART5)
	{
		read=Size;
//		if (Size <= BUFF_SIZE)
//		{
			
			HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buff, BUFF_SIZE); // 接收完毕后重启
			
			sbus_frame_parse(&remoter, rx_buff);

//			memset(rx_buff, 0, BUFF_SIZE);
//		}
//		else  // 接收数据长度大于BUFF_SIZE，错误处理
//		{	
//			HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buff, BUFF_SIZE); // 接收完毕后重启
//			memset(rx_buff, 0, BUFF_SIZE);							   
//		}
		
	}
	if(huart->Instance == UART7)
	{
		HAL_UARTEx_ReceiveToIdle_IT(&huart7,rx_jy,33);
		if(rx_jy[22] == 0x55)
			{
				if(rx_jy[23]==0x53)
				{
					RXDataRoll.DataHEX[0]=rx_jy[24];
					RXDataRoll.DataHEX[1]=rx_jy[25];
					RXDataPitch.DataHEX[0]=rx_jy[26];
					RXDataPitch.DataHEX[1]=rx_jy[27];
					RXDataYaw.DataHEX[0]=rx_jy[28];
					RXDataYaw.DataHEX[1]=rx_jy[29];
				}
			}
	}
	if(huart->Instance == USART1)
	{
		if(flag_u1==0)
		{
			flag_u1=1;
		}
		HAL_UARTEx_ReceiveToIdle_IT(&huart1,rx_middle,6);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buff, BUFF_SIZE); // 接收发生错误后重启
		memset(rx_buff, 0, BUFF_SIZE);							   // 清除接收缓存		
	}
}
