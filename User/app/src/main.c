/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块。
*	文件名称 : main.c
*	版    本 : V1.4
*	说    明 : RS485 MODBUS主站例程（使用的是串口2）。
*              本例程主要讲解MODBUS协议从站的命令处理方法,包含了常用的命令。
*
*********************************************************************************************************
*/

#include "bsp.h"			/* 底层硬件驱动 */
#include "main.h"
#include "modbus_host.h"

/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"MODBUS控制伺服电机"
#define EXAMPLE_DATE	"2021-01-27"
#define DEMO_VER			"6.1"

/* 仅允许本文件内调用的函数声明 */
static void PrintfLogo(void);
void PortScan(void);
void StateScan(void);

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{
	bsp_Init();						/* 硬件初始化 */
  PrintfLogo();					/* 打印例程信息到串口1 */

	/* 进入主程序循环体 */
	while (1)
	{
		bsp_Idle();					/* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */	
	}
}

static void PrintfLogo(void)
{
	printf("STM32_READY");
}

/*
*********************************************************************************************************
*	函 数 名: PortScan
*	功能说明: 扫描各个端口的电平状态并执行动作
*	形    参：无
*	返 回 值: 无
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
	
	/* 急停按钮输入 */
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
				g_tCtrlH.EMR_Stop = 0x01;	//产生急停信号
        printf("9");
				state_EMR_STOP = WAIT_DOWN_1;	//等待急停信号释放
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
	
	/* Y轴到位信号输出 */
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
				g_tCtrlH.Y_finish = 0;	//清零到位信号
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
				g_tCtrlH.Y_finish = 1;	//置位到位信号
        printf("0");
			}				
			else state_Y_FINISH = WAIT_DOWN_1;
		break;
	}
  
  /* X1轴到位信号输出 */
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
				g_tCtrlH.X1_finish = 0;	//清零到位信号
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
				g_tCtrlH.X1_finish = 1;	//置位到位信号
        printf("1");
			}				
			else state_X1_FINISH = WAIT_DOWN_1;
		break;
	}
	
	/* Z1轴到位信号输出 */
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
				g_tCtrlH.Z1_finish = 0;	//清零到位信号
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
				g_tCtrlH.Z1_finish = 1;	//置位到位信号
        printf("2");
			}				
			else state_Z1_FINISH = WAIT_DOWN_1;
		break;
	}
  
  /* X2轴到位信号输出 */
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
				g_tCtrlH.X2_finish = 0;	//清零到位信号
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
				g_tCtrlH.X2_finish = 1;	//置位到位信号
        printf("3");
			}				
			else state_X2_FINISH = WAIT_DOWN_1;
		break;
	}
  
  /* Z2轴到位信号输出 */
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
				g_tCtrlH.Z2_finish = 0;	//清零到位信号
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
				g_tCtrlH.Z2_finish = 1;	//置位到位信号
        printf("4");
			}				
			else state_Z2_FINISH = WAIT_DOWN_1;
		break;
	}
}

