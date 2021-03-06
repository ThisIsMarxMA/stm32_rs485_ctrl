/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块
*	文件名称 : main.h
*	版    本 : V1.4
*	说    明 : 头文件
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#ifndef __MAIN_H_
#define __MAIN_H_
#include "stdint.h"

typedef struct
{
	uint8_t Rxlen;
	char RxBuf[20];
	uint8_t Txlen;
	char TxBuf[20];
}PRINT_MODS_T;

extern PRINT_MODS_T g_tPrint;
void PortScan(void);
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
