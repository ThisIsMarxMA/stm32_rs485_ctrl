/*
*********************************************************************************************************
*
*	模块名称 : EXTI模块
*	文件名称 : exti.c
*	说    明 : 用于配置0-15线的外部中断来读取电机上下限幅，零位信号等
*
*
*********************************************************************************************************
*/

#include "exti.h"
#include "modbus_host.h"

void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI0_IRQHandler(void);
	
/*
*********************************************************************************************************
*	函 数 名: GPIO_Setup
*	功能说明: 配置GPIO口,其中GPIOF0~8,12~15与GPIOG9~11为外部中断输入口.  
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void GPIO_Setup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOC|
                         RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|
                                GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	//下拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIOF
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIOC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|
                                GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		
	GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIOF
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIOB
} 
