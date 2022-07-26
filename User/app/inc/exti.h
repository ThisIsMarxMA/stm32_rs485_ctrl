#ifndef _EXTI_H
#define _EXTI_H
#include "bsp.h"
#include "main.h"

#define Y_FINISH				GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)	
#define X1_FINISH				GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)	
#define Z1_FINISH				GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)	
#define X2_FINISH 		  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)	
#define Z2_FINISH 	    GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2) 
#define X3_FINISH 		  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3)	
#define Z3_FINISH 	    GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) 

#define Y_Alarm         GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_3)
#define X1_Alarm        GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_4)
#define Z1_Alarm        GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_5)
#define X2_Alarm        GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_6)
#define Z2_Alarm        GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_7)
#define X3_Alarm        GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_8)
#define Z3_Alarm        GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_9)

#define EMR_STOP 				GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) 
#define PWR_STATE       GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)
#define BOARD_DETC      GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)
#define OBST_DETC       GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)
#define DOOR_DETC       GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_12)

//#define EMRLIGHT_ON				GPIO_SetBits(GPIOF,GPIO_Pin_0)		  
//#define EMRLIGHT_OFF	    GPIO_ResetBits(GPIOF,GPIO_Pin_0)			//RED

//#define RUNLIGHT_ON			  GPIO_SetBits(GPIOF,GPIO_Pin_1)	
//#define RUNLIGHT_OFF	    GPIO_ResetBits(GPIOF,GPIO_Pin_1)			//GREEN

//#define LASER_ON	  	    GPIO_SetBits(GPIOF,GPIO_Pin_2)		
//#define LASER_OFF	        GPIO_ResetBits(GPIOF,GPIO_Pin_2)			//YELLOW 

//#define Z1BRAKE_RELEASE   GPIO_SetBits(GPIOB,GPIO_Pin_5)
//#define Z1BRAKE_CLOSE     GPIO_ResetBits(GPIOB,GPIO_Pin_5) 

//#define Z2BRAKE_RELEASE   GPIO_SetBits(GPIOB,GPIO_Pin_6)
//#define Z2BRAKE_CLOSE     GPIO_ResetBits(GPIOB,GPIO_Pin_6) 

//#define Z3BRAKE_RELEASE   GPIO_SetBits(GPIOB,GPIO_Pin_7)
//#define Z3BRAKE_CLOSE     GPIO_ResetBits(GPIOB,GPIO_Pin_7) 

//#define YBRAKE_RELEASE    GPIO_SetBits(GPIOB,GPIO_Pin_8)
//#define YBRAKE_CLOSE      GPIO_ResetBits(GPIOB,GPIO_Pin_8) 

//反接
#define EMRLIGHT_OFF				GPIO_SetBits(GPIOF,GPIO_Pin_0)		  
#define EMRLIGHT_ON	        GPIO_ResetBits(GPIOF,GPIO_Pin_0)			//RED

#define RUNLIGHT_OFF			  GPIO_SetBits(GPIOF,GPIO_Pin_1)	
#define RUNLIGHT_ON	        GPIO_ResetBits(GPIOF,GPIO_Pin_1)			//GREEN

#define LASER_OFF	  	      GPIO_SetBits(GPIOF,GPIO_Pin_2)		
#define LASER_ON	          GPIO_ResetBits(GPIOF,GPIO_Pin_2)			//YELLOW 

#define Z1BRAKE_CLOSE       GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define Z1BRAKE_RELEASE     GPIO_ResetBits(GPIOB,GPIO_Pin_5) 

#define Z2BRAKE_CLOSE       GPIO_SetBits(GPIOB,GPIO_Pin_6)
#define Z2BRAKE_RELEASE     GPIO_ResetBits(GPIOB,GPIO_Pin_6) 

#define Z3BRAKE_CLOSE       GPIO_SetBits(GPIOB,GPIO_Pin_7)
#define Z3BRAKE_RELEASE     GPIO_ResetBits(GPIOB,GPIO_Pin_7) 

#define YBRAKE_CLOSE        GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define YBRAKE_RELEASE      GPIO_ResetBits(GPIOB,GPIO_Pin_8) 


void GPIO_Setup(void);	//IO初始化
void EXTIX_Init(void);
#endif

