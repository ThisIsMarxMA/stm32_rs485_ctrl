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
#define EXAMPLE_DATE	"2022-06-28"
#define DEMO_VER			"1.0"

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
void motor_finish(uint8_t* state_axis_FINISH, uint8_t axis)
{
	uint8_t axis_result = 0;
  switch(axis)
  {
    case SERVOY:
      axis_result = Y_FINISH; break;
    case SERVOX1:
      axis_result = X1_FINISH; break;
    case SERVOZ1:
      axis_result = Z1_FINISH; break;
    case SERVOX2:
      axis_result = X2_FINISH; break;
    case SERVOZ2:
      axis_result = Z2_FINISH; break;
    case SERVOX3:
      axis_result = X3_FINISH; break;
    case SERVOZ3:
      axis_result = Z3_FINISH; break;
    default:;    
  }
  
  if(axis_result == 0)
  {
    g_tCtrlH.Motor_finish[axis] = 1;	//��λ��λ�ź�
  }
  else
  {
    g_tCtrlH.Motor_finish[axis] = 0;	//���㵽λ�ź�
    printf("%d", axis);
  }
}

void motor_alarm(uint8_t* state_axis_ALARM, uint8_t axis)
{
	uint8_t axis_result = 0;
  switch(axis)
  {
    case SERVOY:
      axis_result = Y_Alarm; break;
    case SERVOX1:
      axis_result = X1_Alarm; break;
    case SERVOZ1:
      axis_result = Z1_Alarm; break;
    case SERVOX2:
      axis_result = X2_Alarm; break;
    case SERVOZ2:
      axis_result = Z2_Alarm; break;
    case SERVOX3:
      axis_result = X3_Alarm; break;
    case SERVOZ3:
      axis_result = Z3_Alarm; break;
    default:;    
  }
  
  if(axis_result == 0)
  {
    g_tCtrlH.Motor_alarm[axis] = 0;	//�����ź�
  }
  else
  {
    g_tCtrlH.Motor_alarm[axis] = 1;	//��λ�ź�
    printf("%d", axis+0x60);
  }
}

void common_signals(uint8_t* state_signal, uint8_t signal)
{
	uint8_t sig_result = 0;
  switch(signal)
  {
    case Sig_EMR_STOP:
      sig_result = EMR_STOP; break;
    case Sig_PWR_STATE:
      sig_result = PWR_STATE; break;
    case Sig_BOARD_DETC:
      sig_result = BOARD_DETC; break;
    case Sig_OBST_DETC:
      sig_result = OBST_DETC; break;
    case Sig_DOOR_DETC:
      sig_result = DOOR_DETC; break;
    default:;    
  }
  
  if(sig_result == 0)
  {
    g_tCtrlH.signals[signal] = 0x00;
  }
  else
  {
    g_tCtrlH.signals[signal] = 0x01;	//�����ź�
    printf("%d", signal+0xA0);
  }
}

void PortScan(void)
{
	static uint8_t state_Sig[5]  = {IDLE};
	static uint8_t state_FINISH[7] 	 = {IDLE}; 
  static uint8_t state_ALARM[7] 	 = {IDLE};
	
  for(int i = 0; i<Sig_ALL; i++)
  {
    common_signals(&state_Sig[i], i);
  }
  
	for(int i = 0; i<SERVO_ALL; i++)
  {
    /* �ᵽλ�ź����� */
    motor_finish(&state_FINISH[i], i);
    /* �ᱨ���ź����� */
    motor_alarm(&state_ALARM[i], i);
  }
  
}

