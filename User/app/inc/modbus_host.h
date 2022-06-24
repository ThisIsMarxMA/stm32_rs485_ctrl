/*
*********************************************************************************************************
*
*	模块名称 : MODEBUS 通信模块 (主机程序）
*	文件名称 : modbus_host.h
*	版    本 : V1.4
*	说    明 : 头文件
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/
#ifndef __MOSBUS_HOST_H
#define __MOSBUS_HOST_H
#include "bsp.h"
#include "stdint.h"
#include "stm32f4xx_conf.h"

#define HBAUD485		UART3_BAUD
#define H_RX_BUF_SIZE			128
#define H_TX_BUF_SIZE     128

/* RTU 应答代码 */
#define RSP_OK						0			/* 成功 */
#define RSP_ERR_CMD				0x01	/* 不支持的功能码 */
#define RSP_ERR_REG_ADDR	0x02	/* 寄存器地址错误 */
#define RSP_ERR_VALUE			0x03	/* 数据值域错误 */
#define RSP_ERR_WRITE			0x04	/* 写入失败 */
#define RSP_ERR						0x05	/* 未知错误 */

//电机ID
#define SERVO1 			 0x01		//Y轴
#define SERVO2 			 0x02		//X1轴
#define SERVO3 			 0x03		//Z1轴
#define SERVO_ALL		 0x00		

//急停触发代码
#define EMR_BUTTON			0x00
#define EMR_X_UP				0x01
#define EMR_X_DOWN			0x02
#define EMR_Y_UP				0x03
#define EMR_Y_DOWN			0x04
#define EMR_Z_UP				0x05
#define EMR_Z_DOWN			0x06

typedef struct
{
			 uint8_t RxBuf[H_RX_BUF_SIZE];
	__IO uint8_t RxCount;
	__IO uint8_t RxStatus;
	__IO uint8_t RxNewFlag;

	__IO uint8_t RspCode;

			 uint8_t TxBuf[H_TX_BUF_SIZE];
	__IO uint8_t TxCount;
	
	__IO uint8_t SlaveAddr;	/* 保存发送到的从机的地址 */
	
	__IO uint16_t Reg03H;		/* 保存主机发送的寄存器首地址 */
	__IO uint16_t Reg06H;
	__IO uint16_t Reg10H;

	__IO uint8_t RegNum;			/* 寄存器个数 */

	__IO uint8_t fAck03H;		/* 应答命令标志 0 表示执行失败 1表示执行成功 */
	__IO uint8_t fAck06H;		
	__IO uint8_t fAck10H;

	__IO uint16_t P01;				/* 03H 读写保持寄存器 */
	__IO uint16_t P02;
	
}MODH_T; /* MODBUS通信处理缓存区 */

typedef struct
{
			 uint8_t RxBuf[H_RX_BUF_SIZE];
	__IO uint8_t RxCount;
			 uint8_t TxBuf[H_TX_BUF_SIZE];
	__IO uint8_t TxCount;
	
	__IO uint8_t RxNewFlag;
	
	uint8_t SlaveAddr;
}UploadH_T; /* 上位机通讯处理缓存区 */

typedef struct
{
	__IO uint8_t ZaxisLift;		//Z轴抬起信号
	__IO uint8_t XaxisZero;		//X轴零位信号
	__IO uint8_t YaxisZero;		//Y轴零位信号
	__IO uint8_t ZaxisZero;		//Z轴零位信号
	__IO uint8_t EMR_Stop;
	
	__IO int16_t Xinitial;	//(1,1)灯X初始值
	__IO int16_t Yinitial;	//(1,1)灯Y初始值
	__IO int16_t Zinitial;  //Z轴下降的距离
	__IO int16_t ZStop;  		//Z轴禁动位置
	__IO uint8_t rowNum;	//X个数
	__IO uint8_t colNum;	//Y个数
	__IO uint16_t Xgap;	//X间距
	__IO uint16_t Ygap;	//Y间距
	__IO uint8_t Xnumber;	//当前灯坐标
	__IO uint8_t Ynumber;
	__IO uint8_t Xnumber_Old; //记录一下上次的坐标值
	__IO uint8_t Ynumber_Old;	
	__IO uint16_t K_numer;		//K值分子
	__IO uint16_t K_denomi;	//K值分母
	__IO uint8_t Xstate;	//X轴状态
	__IO uint8_t Ystate;	//Y轴状态
	__IO uint8_t Zstate;	//Z轴状态
	__IO int16_t Xlocate;	//X轴位置 0.1mm值
	__IO int16_t Ylocate;
	__IO int16_t Zlocate;
	__IO int16_t Xlocate_Old;	//记录上一次位置 0.1mm值
	__IO int16_t Ylocate_Old;
	__IO int16_t Zlocate_Old;
	__IO int32_t X_value;	//电机转角值
	__IO int32_t Y_value;	//电机转角值
	__IO int32_t Z_value;	//电机转角值
	__IO uint8_t Y_finish;	
  __IO uint8_t X1_finish;	//X轴电机是否到位
	__IO uint8_t Z1_finish;	
  __IO uint8_t X2_finish;	//X轴电机是否到位
	__IO uint8_t Z2_finish;	
  
	__IO int8_t XdirTo0;	//X轴电机转向零点的方向 -1或1
	__IO int8_t YdirTo0;
	__IO int8_t ZdirTo0;
	
	__IO uint8_t FrameID;	//帧ID
	__IO uint8_t ChannelID;	//通道ID

	__IO uint8_t RSP_Code;			//上位机应答代码
	__IO uint8_t UploadH_Lock;	//上位机串口锁
	__IO uint8_t CheckPort;			//每10ms执行PortScan
	__IO uint8_t CheckState;		//每15ms执行StateScan
	__IO uint8_t ZBREAK_Lock;		//Z轴刹车锁
}CtrlH_T;		/* modbus通讯控制结构体 */

void ServoInit(void);
void MODH_Poll(void);
void UploadH_Poll(void);
uint8_t MODH_ReadParam_03H(uint8_t SlaveAddr, uint16_t _reg, uint16_t _num);
uint8_t MODH_WriteParam_06H(uint8_t SlaveAddr, uint16_t _reg, uint16_t _value);
uint8_t MODH_WriteParam_10H(uint8_t SlaveAddr, uint16_t _reg, uint16_t _num, uint8_t *_buf);

void UploadH_ReciveClear(COM_PORT_E _ucPort);
void ZBREAK_Unlock(void);
void ZBREAK_Lock(void);
int8_t ZBREAK_IsLocked(void);
extern MODH_T g_tModH;
extern CtrlH_T g_tCtrlH;
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
