#ifndef _EXTI_H
#define _EXTI_H
#include "bsp.h"
#include "main.h"

#define Y_FINISH				GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9)		//PG9
#define X1_FINISH				GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_10)	//PG10
#define Z1_FINISH				GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_15)	//PG15
#define X2_FINISH 		  GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_7)	//PG7
#define Z2_FINISH 	    GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_8) //PG8

#define EMR_STOP 				GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0) 	//PA0

#define EMRLIGHT_ON			GPIO_SetBits(GPIOF,GPIO_Pin_14);		  
#define EMRLIGHT_OFF		GPIO_ResetBits(GPIOF,GPIO_Pin_14);			//RED

#define RUNLIGHT_ON		  GPIO_SetBits(GPIOF,GPIO_Pin_13);		
#define RUNLIGHT_OFF		GPIO_ResetBits(GPIOF,GPIO_Pin_13);			//GREEN

#define TEMPSTOP_ON	  	GPIO_SetBits(GPIOF,GPIO_Pin_12);		
#define TEMPSTOP_OFF	  GPIO_ResetBits(GPIOF,GPIO_Pin_12);			//YELLOW 

#define ZBRAKE_CLOSE    
#define ZBRAKE_RELEASE  

void GPIO_Setup(void);	//IO≥ı ºªØ
void EXTIX_Init(void);
#endif

