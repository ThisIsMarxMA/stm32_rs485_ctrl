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
#define SERVOY 			   0x00		//Y��
#define SERVOX1 			 0x01		//X1��
#define SERVOZ1 			 0x02		//Z1��
#define SERVOX2 			 0x03		//X2��
#define SERVOZ2 			 0x04		//Z2��
#define SERVOX3 			 0x05		//X3��
#define SERVOZ3 			 0x06		//Z3��
#define SERVO_ALL		   0x07		

#define EMRLIGHT       0x01
#define RUNLIGHT       0x02
#define TEMPSTOP       0x03

//��ͣ��������
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
	__IO uint8_t EMR_Stop;
	

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
  
  __IO int16_t Motor_initial[SERVO_ALL];	//(1,1)���ʼֵ
  __IO uint8_t Motor_axisZero[SERVO_ALL];	//����λ�ź�
	__IO uint8_t Motor_state[SERVO_ALL];	//��״̬
	__IO int16_t Motor_locate[SERVO_ALL];	//��λ�� 0.1mmֵ
	__IO int16_t Motor_locate_Old[SERVO_ALL];	//��¼��һ��λ�� 0.1mmֵ
	__IO int32_t Motor_value[SERVO_ALL];	//���ת��ֵ
	__IO uint8_t Motor_finish[SERVO_ALL];	//�����Ƿ�λ  
  __IO uint8_t Motor_alarm[SERVO_ALL];
	__IO int8_t Motor_dirTo0[SERVO_ALL];	//����ת�����ķ��� -1��1
	__IO uint8_t Z_Lock[SERVO_ALL];		//ɲ����
  __IO uint8_t signals[Sig_ALL];
	
	__IO uint8_t FrameID;	//֡ID
	__IO uint8_t ChannelID;	//ͨ��ID
  __IO uint8_t ObjectID;	//����ID
  __IO uint8_t ObjectState;	//����ID
  

	__IO uint8_t RSP_Code;		//��λ��Ӧ�����
	__IO uint8_t UploadH_Lock;//��λ��������
	__IO uint8_t CheckPort;		//ÿ10msִ��PortScan
	__IO uint8_t CheckState;	//ÿ15msִ��StateScan

}CtrlH_T;		/* modbusͨѶ���ƽṹ�� */

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

/***************************** ���������� www.armfly.com (END OF FILE) *********************************/
