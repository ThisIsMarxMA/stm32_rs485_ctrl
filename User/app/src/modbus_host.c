/*
*********************************************************************************************************
*
*	ģ������ : MODSBUSͨ�ų��� ��������
*	�ļ����� : modbus_host.c
*	��    �� : V1.4
*	˵    �� : ����ͨ�ų���ͨ��Э�����MODBUS
*	�޸ļ�¼ :
*		�汾��  ����        ����    ˵��
*       V1.4   2015-11-28 �޸�Э��
*
*
*********************************************************************************************************
*/
#include "bsp.h"
#include "main.h"
#include "modbus_host.h"

#define TIMEOUT		60		/* �������ʱʱ��, ��λms */
#define NUM				5		/* ѭ�����ʹ��� */

MODH_T g_tModH;
UploadH_T g_tUploadH;
CtrlH_T g_tCtrlH;
uint8_t g_modh_timeout = 0;

static void UploadH_AnalyseFrame(void);
static void UploadH_Read_0x11(void);
static void UploadH_Read_0x12(void);
static void UploadH_Read_0x13(void);
static void UploadH_Read_0x14(void);
static void UploadH_Read_0x15(void);
void UploadH_SendError(void);


/*****************************************����Ϊ����λ��ͨѶ����****************************************/
/*
*********************************************************************************************************
*	�� �� ��: ZBREAK_IsLocked,ZBREAK_Lock,ZBREAK_Unlock
*	����˵��: Z��ɲ�������ͷ���ز�������  
*	��    ��: g_tCtrlH.ZBREAK_Lock
*	�� �� ֵ: ��
*********************************************************************************************************
*/
int8_t ZBREAK_IsLocked(void)
{
	return g_tCtrlH.ZBREAK_Lock;
}	

void ZBREAK_Lock(void)
{
	ZBRAKE_CLOSE;		//ɲ������
	g_tCtrlH.Zstate = 0x00;
	g_tCtrlH.ZBREAK_Lock = 1;
}

void ZBREAK_Unlock(void)
{
	ZBRAKE_RELEASE;	//ɲ���ͷ�
	g_tCtrlH.Zstate = 0x01;
	g_tCtrlH.ZBREAK_Lock = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: SerialUpload_ReciveNew
*	����˵��: ����3�����жϷ���������ñ����������յ�һ���ֽ�ʱ��ִ��һ�α�������
*	��    ��: 
*	�� �� ֵ: 1 ��ʾ������
*********************************************************************************************************
*/
void UploadH_ReciveNew(uint8_t _data)
{
	static uint8_t RxStatus = WAIT_E;
	switch(RxStatus)
	{
		case WAIT_E:
			if(_data == 'E') RxStatus = WAIT_F;
		break;
		case WAIT_F:
			if(_data == 'F') RxStatus = WAIT_G;
			else RxStatus = WAIT_E;
		break;
		case WAIT_G:
			if(_data == 'G') RxStatus = WAIT_H;
			else
			{
				g_tUploadH.RxBuf[g_tUploadH.RxCount++] = _data;
				if(g_tUploadH.RxCount > (H_RX_BUF_SIZE-1))
				{
					g_tUploadH.RxCount = 0;
				}
			}
		break;
		case WAIT_H:
			if(_data == 'H') g_tUploadH.RxNewFlag = 1;
			else g_tUploadH.RxCount = 0;
			RxStatus = WAIT_E;
		break;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: UploadH_ReciveClear
*	����˵��: �����λ��ͨ��COM�������н��յ�����
*	��    ��: 
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void UploadH_ReciveClear(COM_PORT_E _ucPort)
{
	comClearRxFifo(_ucPort);
	g_tUploadH.RxNewFlag = 0;
	g_tUploadH.RxCount = 0;
}

/*
*********************************************************************************************************
*	�� �� ��: UploadH_Poll
*	����˵��: ������λ��ָ�����. 1ms��Ӧʱ�䡣
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void UploadH_Poll(void)
{		
	if(g_tUploadH.RxNewFlag)	//���յ�һ֡��λ������
	{
		if (g_tUploadH.RxCount < 5) goto err_ret;	  
    UploadH_AnalyseFrame();	//�������յ�������
      
err_ret:		
#if 0	/* �˲���Ϊ�˴��ڴ�ӡ���,ʵ�������пɲ�Ҫ */
		g_tPrint.Rxlen = g_tUploadH.RxCount;
		memcpy(g_tPrint.RxBuf, g_tUploadH.RxBuf, g_tUploadH.RxCount);
		bsp_PutMsg(MSG_MODS_RX, 0);
#endif
		UploadH_ReciveClear(Upload_Port);	//��ս��ջ���
    bsp_LedToggle(1);
	}
}

//�������յ�����λ������
static void UploadH_AnalyseFrame(void)
{
	uint8_t temp_buf[4] = {0};
	uint8_t *ptr1,*ptr2;
	
	/* ת���õ�֡ID */
	ptr1 = &g_tUploadH.RxBuf[1];
	ptr2 = temp_buf;
	while((*ptr1 != '/')&&(ptr2 <= &temp_buf[3]))
	{
		*ptr2 = *ptr1;
		ptr1++;
		ptr2++;
	}
	while(ptr2 <= &temp_buf[3]) *(ptr2++) = 0;	//����ʣ��λ
	
	g_tCtrlH.FrameID = atoi((char *)temp_buf);
	
	/* ת���õ�ͨ��ID */
	if(g_tUploadH.RxBuf[4] == 'A')
		g_tCtrlH.ChannelID = 0x00;
	else
		g_tCtrlH.ChannelID = 0x01;
	 											 	
	/* ������Ӧ֡ */
	switch (g_tCtrlH.FrameID)
	{
    case 0x0B:
      UploadH_Read_0x11();
    break;
    
		case 0x0C:	//ɲ�������ͷ�֡ 
			UploadH_Read_0x12();
		break;
		
		case 0x0D:	//�궨��ʼ/����֡ ���Ƶ�
			UploadH_Read_0x13();
		break;
		
		case 0x0E:	//��λ����ͣ֡	���Ƶ����ͣ
			UploadH_Read_0x14();
		break;
		
		case 0x0F:	//��λ����ͣ֡ 
			UploadH_Read_0x15();
		break;
	}		
}

/*
*********************************************************************************************************
*	�� �� ��: mcuRestart
*	����˵��: ��λMCU
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void UploadH_Read_0x11(void)
{
	__set_FAULTMASK(1); //�ر������ж�
	NVIC_SystemReset(); //��λ
}

void UploadH_Read_0x12(void)
{
	if(g_tUploadH.RxBuf[6] == '1'){TEMPSTOP_ON;}
	else if(g_tUploadH.RxBuf[6] == '0'){TEMPSTOP_OFF;}
	
}

void UploadH_Read_0x13(void)
{
	if(g_tUploadH.RxBuf[6] == '1') {RUNLIGHT_ON;}
	else if(g_tUploadH.RxBuf[6] == '0') {RUNLIGHT_OFF;}
}

void UploadH_Read_0x14(void)
{
	if(g_tUploadH.RxBuf[6] == '1'){EMRLIGHT_ON;}	//��ͣ����
	else if(g_tUploadH.RxBuf[6] == '0'){EMRLIGHT_OFF;}	//��ͣ����
}

void UploadH_Read_0x15(void)
{	

	if(g_tUploadH.RxBuf[6] == '1') {ZBREAK_Lock();}
	else if(g_tUploadH.RxBuf[6] == '0') {ZBREAK_Unlock();}
}






