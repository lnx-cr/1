/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "stdio.h"
#include "arm_math.h"

#include <stdio.h>
#include <math.h>

extern float POS2,POS1,POS3,POS4,POS5,POS6,POS7,POS8;
extern float zero_1,zero_2,zero_3,zero_4,zero_5,zero_6,zero_7,zero_8;
extern MOTOR_send cmd_1,cmd_2,cmd_3,cmd_4,cmd_5,cmd_6,cmd_7,cmd_8;  
extern MOTOR_recv data_1,data_2,data_3,data_4,data_5,data_6,data_7,data_8;
extern float hui_1,hui_2,hui_3,hui_4,hui_5,hui_6,hui_7,hui_8;
extern char state,state_fur;
extern float Yaw,Pitch,Roll,Yaw_goal;
extern float lq,lh,rq,rh;
extern float dx;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef struct
{
    struct
    { 
        unsigned short ch0;
        unsigned short ch1;
        unsigned short ch2;
        unsigned short ch3;
        unsigned short ch4;
        unsigned char s1;
        unsigned char s2;
    }rc;
    
    struct 
    {
        int16_t x;
        int16_t y;
        int16_t z;
        unsigned char press_l;
        unsigned char press_r;
    }mouse;
   
    struct
    {
        unsigned short v;
    }key;
		union {
    uint16_t key_code;
    struct
    {
      uint16_t W ;
      uint16_t S ;
      uint16_t A ;
      uint16_t D ;
      uint16_t SHIFT ;
      uint16_t CTRL ;
      uint16_t Q ;
      uint16_t E ;
			uint16_t F ;
			uint16_t G ;
			uint16_t R ;
			uint16_t Z ;
			uint16_t X ;
			uint16_t C ;
			uint16_t V ;
	    uint16_t B ;
    } bit;
  } kb;
		struct
		{
      char shift ;
      char ctrl ;
      char q ;
      char e ;
			char f ;
			char g ;
			char r ;
			char z ;
			char x ;
			char c ;
			char v ;
			char b ;
		}flag;
}RC_Ctl_t;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
