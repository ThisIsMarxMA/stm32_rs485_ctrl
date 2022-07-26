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
#define SERVOY 			   0x00		//Y轴
#define SERVOX1 			 0x01		//X1轴
#define SERVOZ1 			 0x02		//Z1轴
#define SERVOX2 			 0x03		//X2轴
#define SERVOZ2 			 0x04		//Z2轴
#define SERVOX3 			 0x05		//X3轴
#define SERVOZ3 			 0x06		//Z3轴
#define SERVO_ALL		   0x07		

#define EMRLIGHT       0x01
#define RUNLIGHT       0x02
#define TEMPSTOP       0x03

//急停触发代码
#define EMR_BUTTON			0x00
#define EMR_X_UP				0x01
#define EMR_X_DOWN			0x02
#define EMR_Y_UP				0x03
#define EMR_Y_DOWN			0x04
#define EMR_Z_UP				0x05
#define EMR_Z_DOWN			0x06

#define LOCKED      0x01
#define UNLOCKED    0x00
#define LIGHT_ON    0x01
#define LIGHT_OFF   0x00

#define Sig_EMR_STOP  0x00
#define Sig_PWR_STATE  0x01
#define Sig_BOARD_DETC  0x02
#define Sig_OBST_DETC  0x03
#define Sig_DOOR_DETC  0x04
#define Sig_ALL  0x05


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
	__IO uint8_t EMR_Stop;
	

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
  
  __IO int16_t Motor_initial[SERVO_ALL];	//(1,1)轴初始值
  __IO uint8_t Motor_axisZero[SERVO_ALL];	//轴零位信号
	__IO uint8_t Motor_state[SERVO_ALL];	//轴状态
	__IO int16_t Motor_locate[SERVO_ALL];	//轴位置 0.1mm值
	__IO int16_t Motor_locate_Old[SERVO_ALL];	//记录上一次位置 0.1mm值
	__IO int32_t Motor_value[SERVO_ALL];	//电机转角值
	__IO uint8_t Motor_finish[SERVO_ALL];	//轴电机是否到位  
  __IO uint8_t Motor_alarm[SERVO_ALL];
	__IO int8_t Motor_dirTo0[SERVO_ALL];	//轴电机转向零点的方向 -1或1
	__IO uint8_t Z_Lock[SERVO_ALL];		//刹车锁
  __IO uint8_t signals[Sig_ALL];
	
	__IO uint8_t FrameID;	//帧ID
	__IO uint8_t ChannelID;	//通道ID
  __IO uint8_t ObjectID;	//物体ID
  __IO uint8_t ObjectState;	//物体ID
  

	__IO uint8_t RSP_Code;		//上位机应答代码
	__IO uint8_t UploadH_Lock;//上位机串口锁
	__IO uint8_t CheckPort;		//每10ms执行PortScan
	__IO uint8_t CheckState;	//每15ms执行StateScan

}CtrlH_T;		/* modbus通讯控制结构体 */

void ServoInit(void);
void MODH_Poll(void);
void UploadH_Poll(void);
uint8_t MODH_ReadParam_03H(uint8_t SlaveAddr, uint16_t _reg, uint16_t _num);
uint8_t MODH_WriteParam_06H(uint8_t SlaveAddr, uint16_t _reg, uint16_t _value);
uint8_t MODH_WriteParam_10H(uint8_t SlaveAddr, uint16_t _reg, uint16_t _num, uint8_t *_buf);
void UploadH_ReciveClear(COM_PORT_E _ucPort);
void ZBREAK_Unlock(int axis);
void ZBREAK_Lock(int axis);
int8_t ZBREAK_IsLocked(int axis);
extern MODH_T g_tModH;
extern CtrlH_T g_tCtrlH;
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
