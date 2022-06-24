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
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|
                                GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|
                                GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	//����
  GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIOG
	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15|GPIO_Pin_14|GPIO_Pin_13|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		
	GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIOF
} 
