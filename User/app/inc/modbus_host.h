/*
*********************************************************************************************************
*
*	ģ������ : MODEBUS ͨ��ģ�� (��������
*	�ļ����� : modbus_host.h
*	��    �� : V1.4
*	˵    �� : ͷ�ļ�
*
*	Copyright (C), 2015-2016, ���������� www.armfly.com
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

/* RTU Ӧ����� */
#define RSP_OK						0			/* �ɹ� */
#define RSP_ERR_CMD				0x01	/* ��֧�ֵĹ����� */
#define RSP_ERR_REG_ADDR	0x02	/* �Ĵ�����ַ���� */
#define RSP_ERR_VALUE			0x03	/* ����ֵ����� */
#define RSP_ERR_WRITE			0x04	/* д��ʧ�� */
#define RSP_ERR						0x05	/* δ֪���� */

//���ID
#define SERVO1 			 0x01		//Y��
#define SERVO2 			 0x02		//X1��
#define SERVO3 			 0x03		//Z1��
#define SERVO_ALL		 0x00		

//��ͣ��������
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
	
	__IO uint8_t SlaveAddr;	/* ���淢�͵��Ĵӻ��ĵ�ַ */
	
	__IO uint16_t Reg03H;		/* �����������͵ļĴ����׵�ַ */
	__IO uint16_t Reg06H;
	__IO uint16_t Reg10H;

	__IO uint8_t RegNum;			/* �Ĵ������� */

	__IO uint8_t fAck03H;		/* Ӧ�������־ 0 ��ʾִ��ʧ�� 1��ʾִ�гɹ� */
	__IO uint8_t fAck06H;		
	__IO uint8_t fAck10H;

	__IO uint16_t P01;				/* 03H ��д���ּĴ��� */
	__IO uint16_t P02;
	
}MODH_T; /* MODBUSͨ�Ŵ������� */

typedef struct
{
			 uint8_t RxBuf[H_RX_BUF_SIZE];
	__IO uint8_t RxCount;
			 uint8_t TxBuf[H_TX_BUF_SIZE];
	__IO uint8_t TxCount;
	
	__IO uint8_t RxNewFlag;
	
	uint8_t SlaveAddr;
}UploadH_T; /* ��λ��ͨѶ�������� */

typedef struct
{
	__IO uint8_t ZaxisLift;		//Z��̧���ź�
	__IO uint8_t XaxisZero;		//X����λ�ź�
	__IO uint8_t YaxisZero;		//Y����λ�ź�
	__IO uint8_t ZaxisZero;		//Z����λ�ź�
	__IO uint8_t EMR_Stop;
	
	__IO int16_t Xinitial;	//(1,1)��X��ʼֵ
	__IO int16_t Yinitial;	//(1,1)��Y��ʼֵ
	__IO int16_t Zinitial;  //Z���½��ľ���
	__IO int16_t ZStop;  		//Z�����λ��
	__IO uint8_t rowNum;	//X����
	__IO uint8_t colNum;	//Y����
	__IO uint16_t Xgap;	//X���
	__IO uint16_t Ygap;	//Y���
	__IO uint8_t Xnumber;	//��ǰ������
	__IO uint8_t Ynumber;
	__IO uint8_t Xnumber_Old; //��¼һ���ϴε�����ֵ
	__IO uint8_t Ynumber_Old;	
	__IO uint16_t K_numer;		//Kֵ����
	__IO uint16_t K_denomi;	//Kֵ��ĸ
	__IO uint8_t Xstate;	//X��״̬
	__IO uint8_t Ystate;	//Y��״̬
	__IO uint8_t Zstate;	//Z��״̬
	__IO int16_t Xlocate;	//X��λ�� 0.1mmֵ
	__IO int16_t Ylocate;
	__IO int16_t Zlocate;
	__IO int16_t Xlocate_Old;	//��¼��һ��λ�� 0.1mmֵ
	__IO int16_t Ylocate_Old;
	__IO int16_t Zlocate_Old;
	__IO int32_t X_value;	//���ת��ֵ
	__IO int32_t Y_value;	//���ת��ֵ
	__IO int32_t Z_value;	//���ת��ֵ
	__IO uint8_t Y_finish;	
  __IO uint8_t X1_finish;	//X�����Ƿ�λ
	__IO uint8_t Z1_finish;	
  __IO uint8_t X2_finish;	//X�����Ƿ�λ
	__IO uint8_t Z2_finish;	
  
	__IO int8_t XdirTo0;	//X����ת�����ķ��� -1��1
	__IO int8_t YdirTo0;
	__IO int8_t ZdirTo0;
	
	__IO uint8_t FrameID;	//֡ID
	__IO uint8_t ChannelID;	//ͨ��ID

	__IO uint8_t RSP_Code;			//��λ��Ӧ�����
	__IO uint8_t UploadH_Lock;	//��λ��������
	__IO uint8_t CheckPort;			//ÿ10msִ��PortScan
	__IO uint8_t CheckState;		//ÿ15msִ��StateScan
	__IO uint8_t ZBREAK_Lock;		//Z��ɲ����
}CtrlH_T;		/* modbusͨѶ���ƽṹ�� */

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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
