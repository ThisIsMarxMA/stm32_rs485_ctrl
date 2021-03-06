/*
*********************************************************************************************************
*
*	模块名称 : MODSBUS通信程序 （主机）
*	文件名称 : modbus_host.c
*	版    本 : V1.4
*	说    明 : 无线通信程序。通信协议基于MODBUS
*	修改记录 :
*		版本号  日期        作者    说明
*       V1.4   2015-11-28 修改协议
*
*
*********************************************************************************************************
*/
#include "bsp.h"
#include "main.h"
#include "modbus_host.h"

#define TIMEOUT		60		/* 接收命令超时时间, 单位ms */
#define NUM				5		/* 循环发送次数 */

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


/*****************************************以下为与上位机通讯部分****************************************/
/*
*********************************************************************************************************
*	函 数 名: ZBREAK_IsLocked,ZBREAK_Lock,ZBREAK_Unlock
*	功能说明: Z轴刹车锁紧释放相关操作函数  
*	形    参: g_tCtrlH.ZBREAK_Lock
*	返 回 值: 无
*********************************************************************************************************
*/
int8_t ZBREAK_IsLocked(void)
{
	return g_tCtrlH.ZBREAK_Lock;
}	

void ZBREAK_Lock(void)
{
	ZBRAKE_CLOSE;		//刹车锁紧
	g_tCtrlH.Zstate = 0x00;
	g_tCtrlH.ZBREAK_Lock = 1;
}

void ZBREAK_Unlock(void)
{
	ZBRAKE_RELEASE;	//刹车释放
	g_tCtrlH.Zstate = 0x01;
	g_tCtrlH.ZBREAK_Lock = 0;
}

/*
*********************************************************************************************************
*	函 数 名: SerialUpload_ReciveNew
*	功能说明: 串口3接收中断服务程序会调用本函数。当收到一个字节时，执行一次本函数。
*	形    参: 
*	返 回 值: 1 表示有数据
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
*	函 数 名: UploadH_ReciveClear
*	功能说明: 清除上位机通信COM口中所有接收的数据
*	形    参: 
*	返 回 值: 无
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
*	函 数 名: UploadH_Poll
*	功能说明: 接收上位机指令并解析. 1ms响应时间。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void UploadH_Poll(void)
{		
	if(g_tUploadH.RxNewFlag)	//接收到一帧上位机命令
	{
		if (g_tUploadH.RxCount < 5) goto err_ret;	  
    UploadH_AnalyseFrame();	//分析接收到的命令
      
err_ret:		
#if 0	/* 此部分为了串口打印结果,实际运用中可不要 */
		g_tPrint.Rxlen = g_tUploadH.RxCount;
		memcpy(g_tPrint.RxBuf, g_tUploadH.RxBuf, g_tUploadH.RxCount);
		bsp_PutMsg(MSG_MODS_RX, 0);
#endif
		UploadH_ReciveClear(Upload_Port);	//清空接收缓存
    bsp_LedToggle(1);
	}
}

//分析接收到的上位机命令
static void UploadH_AnalyseFrame(void)
{
	uint8_t temp_buf[4] = {0};
	uint8_t *ptr1,*ptr2;
	
	/* 转换得到帧ID */
	ptr1 = &g_tUploadH.RxBuf[1];
	ptr2 = temp_buf;
	while((*ptr1 != '/')&&(ptr2 <= &temp_buf[3]))
	{
		*ptr2 = *ptr1;
		ptr1++;
		ptr2++;
	}
	while(ptr2 <= &temp_buf[3]) *(ptr2++) = 0;	//清零剩余位
	
	g_tCtrlH.FrameID = atoi((char *)temp_buf);
	
	/* 转换得到通道ID */
	if(g_tUploadH.RxBuf[4] == 'A')
		g_tCtrlH.ChannelID = 0x00;
	else
		g_tCtrlH.ChannelID = 0x01;
	 											 	
	/* 处理相应帧 */
	switch (g_tCtrlH.FrameID)
	{
    case 0x0B:
      UploadH_Read_0x11();
    break;
    
		case 0x0C:	//刹车锁紧释放帧 
			UploadH_Read_0x12();
		break;
		
		case 0x0D:	//标定开始/结束帧 控制灯
			UploadH_Read_0x13();
		break;
		
		case 0x0E:	//上位机急停帧	控制电机急停
			UploadH_Read_0x14();
		break;
		
		case 0x0F:	//上位机暂停帧 
			UploadH_Read_0x15();
		break;
	}		
}

/*
*********************************************************************************************************
*	函 数 名: mcuRestart
*	功能说明: 软复位MCU
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void UploadH_Read_0x11(void)
{
	__set_FAULTMASK(1); //关闭所有中断
	NVIC_SystemReset(); //复位
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
	if(g_tUploadH.RxBuf[6] == '1'){EMRLIGHT_ON;}	//急停灯亮
	else if(g_tUploadH.RxBuf[6] == '0'){EMRLIGHT_OFF;}	//急停灯灭
}

void UploadH_Read_0x15(void)
{	

	if(g_tUploadH.RxBuf[6] == '1') {ZBREAK_Lock();}
	else if(g_tUploadH.RxBuf[6] == '0') {ZBREAK_Unlock();}
}






