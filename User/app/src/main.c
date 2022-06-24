/*
*********************************************************************************************************
*
*	ģ������ : ������ģ�顣
*	�ļ����� : main.c
*	��    �� : V1.4
*	˵    �� : RS485 MODBUS��վ���̣�ʹ�õ��Ǵ���2����
*              ��������Ҫ����MODBUSЭ���վ���������,�����˳��õ����
*
*********************************************************************************************************
*/

#include "bsp.h"			/* �ײ�Ӳ������ */
#include "main.h"
#include "modbus_host.h"

/* ���������������̷������� */
#define EXAMPLE_NAME	"MODBUS�����ŷ����"
#define EXAMPLE_DATE	"2021-01-27"
#define DEMO_VER			"6.1"

/* �������ļ��ڵ��õĺ������� */
static void PrintfLogo(void);
void PortScan(void);
void StateScan(void);

/*
*********************************************************************************************************
*	�� �� ��: main
*	����˵��: c�������
*	��    �Σ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
int main(void)
{
	bsp_Init();						/* Ӳ����ʼ�� */
  PrintfLogo();					/* ��ӡ������Ϣ������1 */

	/* ����������ѭ���� */
	while (1)
	{
		bsp_Idle();					/* ���������bsp.c�ļ����û������޸��������ʵ��CPU���ߺ�ι�� */	
	}
}

static void PrintfLogo(void)
{
	printf("STM32_READY");
}

/*
*********************************************************************************************************
*	�� �� ��: PortScan
*	����˵��: ɨ������˿ڵĵ�ƽ״̬��ִ�ж���
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void PortScan(void)
{
	static uint8_t state_EMR_STOP  = IDLE;
	static uint8_t state_Y_FINISH 	 = IDLE;
  static uint8_t state_X1_FINISH 	 = IDLE;
	static uint8_t state_Z1_FINISH 	 = IDLE;
  static uint8_t state_X2_FINISH 	 = IDLE;
	static uint8_t state_Z2_FINISH 	 = IDLE;
	
	/* ��ͣ��ť���� */
	switch(state_EMR_STOP)
	{
		case IDLE:
			if(EMR_STOP == 1) state_EMR_STOP = WAIT_UP_1;
		break;
		case WAIT_UP_1:
			if(EMR_STOP == 1) state_EMR_STOP = WAIT_UP_2;
			else state_EMR_STOP = IDLE;
		break;
		case WAIT_UP_2:
			if(EMR_STOP == 1)
			{
				g_tCtrlH.EMR_Stop = 0x01;	//������ͣ�ź�
        printf("9");
				state_EMR_STOP = WAIT_DOWN_1;	//�ȴ���ͣ�ź��ͷ�
			}
			else state_EMR_STOP = IDLE;
		break;			
		case WAIT_DOWN_1:
			if(EMR_STOP == 0) state_EMR_STOP = WAIT_DOWN_2_1;
		break;
		case WAIT_DOWN_2_1:
			if(EMR_STOP == 0) state_EMR_STOP = WAIT_DOWN_1_1;
		break;
		case WAIT_DOWN_1_1:
			if(EMR_STOP == 0) state_EMR_STOP = WAIT_DOWN_1_2;
		break;
		case WAIT_DOWN_1_2:
			if(EMR_STOP == 0) state_EMR_STOP = WAIT_DOWN_2;
		break;
		case WAIT_DOWN_2:
			if(EMR_STOP == 0)
			{
				g_tCtrlH.EMR_Stop = 0x00;
				state_EMR_STOP = IDLE;
			}				
			else state_EMR_STOP = WAIT_DOWN_1;
		break;
	}
	
	/* Y�ᵽλ�ź���� */
	switch(state_Y_FINISH)
	{
		case IDLE:
			if(Y_FINISH == 1) state_Y_FINISH = WAIT_UP_1;
		break;
		case WAIT_UP_1:
			if(Y_FINISH == 1) state_Y_FINISH = WAIT_UP_2;
			else state_Y_FINISH = IDLE;
		break;
		case WAIT_UP_2:
			if(Y_FINISH == 1)
			{
				state_Y_FINISH = WAIT_DOWN_1;
				g_tCtrlH.Y_finish = 0;	//���㵽λ�ź�
			}
			else state_Y_FINISH = IDLE;
		break;			
		case WAIT_DOWN_1:
			if(Y_FINISH == 0) state_Y_FINISH = WAIT_DOWN_2;
		break;
		case WAIT_DOWN_2:
			if(Y_FINISH == 0)
			{
				state_Y_FINISH = IDLE;
				g_tCtrlH.Y_finish = 1;	//��λ��λ�ź�
        printf("0");
			}				
			else state_Y_FINISH = WAIT_DOWN_1;
		break;
	}
  
  /* X1�ᵽλ�ź���� */
	switch(state_X1_FINISH)
	{
		case IDLE:
			if(X1_FINISH == 1) state_X1_FINISH = WAIT_UP_1;
		break;
		case WAIT_UP_1:
			if(X1_FINISH == 1) state_X1_FINISH = WAIT_UP_2;
			else state_X1_FINISH = IDLE;
		break;
		case WAIT_UP_2:
			if(X1_FINISH == 1)
			{
				state_X1_FINISH = WAIT_DOWN_1;
				g_tCtrlH.X1_finish = 0;	//���㵽λ�ź�
			}
			else state_X1_FINISH = IDLE;
		break;			
		case WAIT_DOWN_1:
			if(X1_FINISH == 0) state_X1_FINISH = WAIT_DOWN_2;
		break;
		case WAIT_DOWN_2:
			if(X1_FINISH == 0)
			{
				state_X1_FINISH = IDLE;
				g_tCtrlH.X1_finish = 1;	//��λ��λ�ź�
        printf("1");
			}				
			else state_X1_FINISH = WAIT_DOWN_1;
		break;
	}
	
	/* Z1�ᵽλ�ź���� */
	switch(state_Z1_FINISH)
	{
		case IDLE:
			if(Z1_FINISH == 1) state_Z1_FINISH = WAIT_UP_1;
		break;
		case WAIT_UP_1:
			if(Z1_FINISH == 1) state_Z1_FINISH = WAIT_UP_2;
			else state_Z1_FINISH = IDLE;
		break;
		case WAIT_UP_2:
			if(Z1_FINISH == 1)
			{
				state_Z1_FINISH = WAIT_DOWN_1;
				g_tCtrlH.Z1_finish = 0;	//���㵽λ�ź�
			}
			else state_Z1_FINISH = IDLE;
		break;			
		case WAIT_DOWN_1:
			if(Z1_FINISH == 0) state_Z1_FINISH = WAIT_DOWN_2;
		break;
		case WAIT_DOWN_2:
			if(Z1_FINISH == 0)
			{
				state_Z1_FINISH = IDLE;
				g_tCtrlH.Z1_finish = 1;	//��λ��λ�ź�
        printf("2");
			}				
			else state_Z1_FINISH = WAIT_DOWN_1;
		break;
	}
  
  /* X2�ᵽλ�ź���� */
	switch(state_X2_FINISH)
	{
		case IDLE:
			if(X2_FINISH == 1) state_X2_FINISH = WAIT_UP_1;
		break;
		case WAIT_UP_1:
			if(X2_FINISH == 1) state_X2_FINISH = WAIT_UP_2;
			else state_X1_FINISH = IDLE;
		break;
		case WAIT_UP_2:
			if(X2_FINISH == 1)
			{
				state_X2_FINISH = WAIT_DOWN_1;
				g_tCtrlH.X2_finish = 0;	//���㵽λ�ź�
			}
			else state_X2_FINISH = IDLE;
		break;			
		case WAIT_DOWN_1:
			if(X2_FINISH == 0) state_X2_FINISH = WAIT_DOWN_2;
		break;
		case WAIT_DOWN_2:
			if(X2_FINISH == 0)
			{
				state_X2_FINISH = IDLE;
				g_tCtrlH.X2_finish = 1;	//��λ��λ�ź�
        printf("3");
			}				
			else state_X2_FINISH = WAIT_DOWN_1;
		break;
	}
  
  /* Z2�ᵽλ�ź���� */
	switch(state_Z2_FINISH)
	{
		case IDLE:
			if(Z2_FINISH == 1) state_Z2_FINISH = WAIT_UP_1;
		break;
		case WAIT_UP_1:
			if(Z2_FINISH == 1) state_Z2_FINISH = WAIT_UP_2;
			else state_Z2_FINISH = IDLE;
		break;
		case WAIT_UP_2:
			if(Z2_FINISH == 1)
			{
				state_Z2_FINISH = WAIT_DOWN_1;
				g_tCtrlH.Z2_finish = 0;	//���㵽λ�ź�
			}
			else state_Z2_FINISH = IDLE;
		break;			
		case WAIT_DOWN_1:
			if(Z2_FINISH == 0) state_Z2_FINISH = WAIT_DOWN_2;
		break;
		case WAIT_DOWN_2:
			if(Z2_FINISH == 0)
			{
				state_Z2_FINISH = IDLE;
				g_tCtrlH.Z2_finish = 1;	//��λ��λ�ź�
        printf("4");
			}				
			else state_Z2_FINISH = WAIT_DOWN_1;
		break;
	}
}

