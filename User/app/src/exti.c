/*
*********************************************************************************************************
*
*	ģ������ : EXTIģ��
*	�ļ����� : exti.c
*	˵    �� : ��������0-15�ߵ��ⲿ�ж�����ȡ��������޷�����λ�źŵ�
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
*	�� �� ��: GPIO_Setup
*	����˵��: ����GPIO��,����GPIOF0~8,12~15��GPIOG9~11Ϊ�ⲿ�ж������.  
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void GPIO_Setup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOC|
                         RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|
                                GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIOF
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|
                                GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB
  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		
	GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIOF
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOB
} 
