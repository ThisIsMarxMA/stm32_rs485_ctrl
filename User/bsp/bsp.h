/*
*********************************************************************************************************
*
*	ģ������ : BSPģ��
*	�ļ����� : bsp.h
*	˵    �� : ���ǵײ�����ģ�����е�h�ļ��Ļ����ļ��� Ӧ�ó���ֻ�� #include bsp.h ���ɣ�
*			  ����Ҫ#include ÿ��ģ��� h �ļ�
*
*	Copyright (C), 2013-2014, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef _BSP_H
#define _BSP_H

#define STM32_V5
//#define STM32_X3

/* ����Ƿ����˿������ͺ� */
#if !defined (STM32_V5) && !defined (STM32_X3)
	#error "Please define the board model : STM32_X3 or STM32_V5"
#endif

/* ���� BSP �汾�� */
#define __STM32F1_BSP_VERSION		"1.1"

/* CPU����ʱִ�еĺ��� */
//#define CPU_IDLE()		bsp_Idle()

/* ����ȫ���жϵĺ� */
#define ENABLE_INT()	__set_PRIMASK(0)	/* ʹ��ȫ���ж� */
#define DISABLE_INT()	__set_PRIMASK(1)	/* ��ֹȫ���ж� */

/* ���������ڵ��Խ׶��Ŵ� */
#define BSP_Printf		printf
//#define BSP_Printf(...)

#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef TRUE
	#define TRUE  1
#endif

#ifndef FALSE
	#define FALSE 0
#endif

typedef enum
{
	IDLE 					= 0x00,
	WAIT_UP_1 		= 0x01,
	WAIT_UP_1_1 	= 0x02,
	WAIT_UP_1_2 	= 0x03,
	WAIT_UP_2 		= 0x04,
	WAIT_DOWN_1 	= 0x05,
	WAIT_DOWN_2 	= 0x06,
	WAIT_DOWN_1_1 = 0x07,
	WAIT_DOWN_1_2 = 0x08,
	WAIT_DOWN_2_1 = 0x09,
	WAIT_UP_1_2_3 = 0x10,
	WAIT_DOWN_1_1_3 = 0x11
}State_TypeDef;

typedef enum
{
	WAIT_E 		= 0x01,
	WAIT_F 		= 0x02,
	WAIT_G 		= 0x03,
	WAIT_H 		= 0x04,
}RecState_TypeDef;

/*
	EXTI9_5_IRQHandler ���жϷ�������ɢ�ڼ��������� bsp�ļ��С�
	��Ҫ���ϵ� stm32f4xx_it.c �С�
	���������б�ʾEXTI9_5_IRQHandler��ں������зŵ� stm32f4xx_it.c��
*/
//#define EXTI9_5_ISR_MOVE_OUT

/* ͨ��ȡ��ע�ͻ������ע�͵ķ�ʽ�����Ƿ�����ײ�����ģ�� */
#include "bsp_uart_fifo.h"
#include "bsp_led.h"
#include "bsp_timer.h"
#include "bsp_watchdog.h"
#include "bsp_spi_flash.h"
#include "bsp_spi_bus.h"

#include "bsp_user_lib.h"
#include "bsp_msg.h"
#include "exti.h"

/* �ṩ������C�ļ����õĺ��� */
void bsp_Init(void);
void bsp_Idle(void);
extern void PortScan(void);
extern void StateScan(void);
#endif

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
