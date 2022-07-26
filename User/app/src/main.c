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

#define MF_ON   0x00 
#define MF_OFF  0x01
#define MA_ON   0x00
#define MA_OFF  0x01
#define CS_ON   0x00
#define CS_OFF  0x01


/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"MODBUS控制伺服电机"
#define EXAMPLE_DATE	"2022-07-26"
#define DEMO_VER			"3.0"

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
  
  for(int i=0;i<SERVO_ALL;i++) 
  {
    g_tCtrlH.Motor_finish[i] = 0x01;
    g_tCtrlH.Motor_alarm[i] = 0x00;
  }
  
  for (int i=0;i<Sig_ALL;i++)
  {
    g_tCtrlH.signals[i] = 0x00;
  }
  
  EMRLIGHT_OFF;
  RUNLIGHT_OFF;
  LASER_OFF;
  Z1BRAKE_CLOSE;
  Z2BRAKE_CLOSE;
  Z3BRAKE_CLOSE;
  YBRAKE_CLOSE;
  
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

  switch(*state_axis_FINISH)
  {
    case IDLE:
      if(axis_result == MF_ON) *state_axis_FINISH = WAIT_UP_1;
    break;
    case WAIT_UP_1:
      if(axis_result == MF_ON) *state_axis_FINISH = WAIT_UP_2;
      else *state_axis_FINISH = IDLE;
    break;
    case WAIT_UP_2:
      if(axis_result == MF_ON)
      {
        *state_axis_FINISH = WAIT_DOWN_1;
        g_tCtrlH.Motor_finish[axis] = 0x00;  //清零到位信号
        printf("%02X", axis);
      }
      else *state_axis_FINISH = IDLE;
    break;      
    case WAIT_DOWN_1:
      if(axis_result == MF_OFF) *state_axis_FINISH = WAIT_DOWN_2;
    break;
    case WAIT_DOWN_2:
      if(axis_result == MF_OFF)
      {
        *state_axis_FINISH = IDLE;
        g_tCtrlH.Motor_finish[axis] = 0x01;  //置位到位信号
      }       
      else *state_axis_FINISH = WAIT_DOWN_1;
    break;
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

  switch(*state_axis_ALARM)
  {
    case IDLE:
      if(axis_result == MA_ON) *state_axis_ALARM = WAIT_UP_1;
    break;
    case WAIT_UP_1:
      if(axis_result == MA_ON) *state_axis_ALARM = WAIT_UP_2;
      else *state_axis_ALARM = IDLE;
    break;
    case WAIT_UP_2:
      if(axis_result == MA_ON)
      {
        *state_axis_ALARM = WAIT_DOWN_1;
        g_tCtrlH.Motor_alarm[axis] = 0x01; //报警信号
      }
      else *state_axis_ALARM = IDLE;
    break;      
    case WAIT_DOWN_1:
      if(axis_result == MA_OFF) *state_axis_ALARM = WAIT_DOWN_2;
    break;
    case WAIT_DOWN_2:
      if(axis_result == MA_OFF)
      {
        *state_axis_ALARM = IDLE;
        g_tCtrlH.Motor_alarm[axis] = 0x00; //报警信号
      }       
      else *state_axis_ALARM = WAIT_DOWN_1;
    break;
  }
  
  if(g_tCtrlH.Motor_alarm[axis] == 0x01)	printf("%02X", axis+0x60);
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

  switch(*state_signal)
  {
    case IDLE:
      if(sig_result == CS_ON) *state_signal = WAIT_UP_1;
    break;
    case WAIT_UP_1:
      if(sig_result == CS_ON) *state_signal = WAIT_UP_2;
      else *state_signal = IDLE;
    break;
    case WAIT_UP_2:
      if(sig_result == CS_ON)
      {
        *state_signal = WAIT_DOWN_1;  //等待信号释放
        g_tCtrlH.signals[signal] = 0x01;  //产生信号
//        printf("%02X", signal+0xA0);
      }
      else *state_signal = IDLE;
    break;      
    case WAIT_DOWN_1:
      if(sig_result == CS_OFF) *state_signal = WAIT_DOWN_2_1;
    break;
    case WAIT_DOWN_2_1:
      if(sig_result == CS_OFF) *state_signal = WAIT_DOWN_1_1;
    break;
    case WAIT_DOWN_1_1:
      if(sig_result == CS_OFF) *state_signal = WAIT_DOWN_1_2;
    break;
    case WAIT_DOWN_1_2:
      if(sig_result == CS_OFF) *state_signal = WAIT_DOWN_2;
    break;
    case WAIT_DOWN_2:
      if(sig_result == CS_OFF)
      {
        g_tCtrlH.signals[signal] = 0x00;
        *state_signal = IDLE;
      }       
      else *state_signal = WAIT_DOWN_1;
    break;
  }
  
  if(g_tCtrlH.signals[signal] == 0x01) printf("%02X", signal+0xA0);
}

void PortScan(void)
{
	static uint8_t state_Sig[Sig_ALL]  = {IDLE};
	static uint8_t state_FINISH[SERVO_ALL] 	 = {IDLE}; 
  static uint8_t state_ALARM[SERVO_ALL] 	 = {IDLE};
	
  for(int i = 0; i<Sig_ALL; i++)
  {
    common_signals(&state_Sig[i], i);
  }
  
	for(int i = 0; i<SERVO_ALL; i++)
  {
    /* 轴到位信号输入 */
    motor_finish(&state_FINISH[i], i);
    /* 轴报警信号输入 */
    motor_alarm(&state_ALARM[i], i);
  }
  
}

