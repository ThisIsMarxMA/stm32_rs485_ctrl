/*
*********************************************************************************************************
*
*	模块名称 : 串口中断+FIFO驱动模块
*	文件名称 : bsp_uart_fifo.c
*	版    本 : V1.0
*	说    明 : 采用串口中断+FIFO模式实现多个串口的同时访问
*	修改记录 :
*		版本号  日期       作者    说明
*		V1.0    2013-02-01 armfly  正式发布
*		V1.1    2013-06-09 armfly  FiFo结构增加TxCount成员变量，方便判断缓冲区满; 增加 清FiFo的函数
*		V1.2	2014-09-29 armfly  增加RS485 MODBUS接口。接收到新字节后，直接执行回调函数。
*		V1.3	2015-07-23 armfly  增加 UART_T 结构的读写指针几个成员变量必须增加 __IO 修饰,否则优化后
*					会导致串口发送函数死机。
*		V1.4	2015-08-04 armfly  解决UART4配置bug  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART1);
*		V1.5	2015-10-08 armfly  增加修改波特率的接口函数
*
*
*********************************************************************************************************
*/

#include "bsp.h"

/* 定义每个串口结构体变量 */
#if UART1_FIFO_EN == 1
	static UART_T g_tUart1;
	static uint8_t g_TxBuf1[UART1_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf1[UART1_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART2_FIFO_EN == 1
	static UART_T g_tUart2;
	static uint8_t g_TxBuf2[UART2_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf2[UART2_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART3_FIFO_EN == 1
	static UART_T g_tUart3;
	static uint8_t g_TxBuf3[UART3_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf3[UART3_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

#if UART5_FIFO_EN == 1
	static UART_T g_tUart5;
	static uint8_t g_TxBuf5[UART5_TX_BUF_SIZE];		/* 发送缓冲区 */
	static uint8_t g_RxBuf5[UART5_RX_BUF_SIZE];		/* 接收缓冲区 */
#endif

static void UartVarInit(void);
static void InitHardUart(void);
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen);
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte);
static void UartIRQ(UART_T *_pUart);
static void ConfigUartNVIC(void);
void RS485_InitTXE(void);

/*
*********************************************************************************************************
*	函 数 名: bsp_InitUart
*	功能说明: 初始化串口硬件，并对全局变量赋初值.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitUart(void)
{
	UartVarInit();		/* 必须先初始化全局变量,再配置硬件 */
	InitHardUart();		/* 配置串口的硬件参数(波特率等) */
//	RS485_InitTXE();	/* 配置RS485芯片的发送使能硬件，配置为推挽输出 */
	ConfigUartNVIC();	/* 配置串口中断 */
}

/*
*********************************************************************************************************
*	函 数 名: ComToUart
*	功能说明: 将COM端口号转换为UART指针
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: uart指针
*********************************************************************************************************
*/
UART_T *ComToUart(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
		#if UART1_FIFO_EN == 1
			return &g_tUart1;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM2)
	{
		#if UART2_FIFO_EN == 1
			return &g_tUart2;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM3)
	{
		#if UART3_FIFO_EN == 1
			return &g_tUart3;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM5)
	{
		#if UART5_FIFO_EN == 1
			return &g_tUart5;
		#else
			return 0;
		#endif
	}
	else return 0;		/* 不做任何处理 */
}

/*
*********************************************************************************************************
*	函 数 名: ComToUart
*	功能说明: 将COM端口号转换为 USART_TypeDef* USARTx
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: USART_TypeDef*,  USART1, USART2, USART3, UART4, UART5
*********************************************************************************************************
*/
USART_TypeDef *ComToUSARTx(COM_PORT_E _ucPort)
{
	if (_ucPort == COM1)
	{
		#if UART1_FIFO_EN == 1
			return USART1;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM2)
	{
		#if UART2_FIFO_EN == 1
			return USART2;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM3)
	{
		#if UART3_FIFO_EN == 1
			return USART3;
		#else
			return 0;
		#endif
	}
	else if (_ucPort == COM5)
	{
		#if UART5_FIFO_EN == 1
			return UART5;
		#else
			return 0;
		#endif
	}
	else return 0;	/* 不做任何处理 */
}

/*
*********************************************************************************************************
*	函 数 名: comSendBuf
*	功能说明: 向串口发送一组数据。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM6)
*			  		_ucaBuf: 待发送的数据缓冲区
*			  _		usLen : 数据长度
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendBuf(COM_PORT_E _ucPort, uint8_t *_ucaBuf, uint16_t _usLen)
{
	UART_T *pUart;
	pUart = ComToUart(_ucPort);
	if (pUart == 0)
		return;

	if (pUart->SendBefor != 0)
		pUart->SendBefor();		/* 如果是RS485通信，可以在这个函数中将RS485设置为发送模式 */

	UartSend(pUart, _ucaBuf, _usLen);
}

/*
*********************************************************************************************************
*	函 数 名: comSendChar
*	功能说明: 向串口发送1个字节。数据放到发送缓冲区后立即返回，由中断服务程序在后台完成发送
*	形    参: _ucPort: 端口号(COM1 - COM6)
*			  _ucByte: 待发送的数据
*	返 回 值: 无
*********************************************************************************************************
*/
void comSendChar(COM_PORT_E _ucPort, uint8_t _ucByte)
{
	comSendBuf(_ucPort, &_ucByte, 1);
}

/*
*********************************************************************************************************
*	函 数 名: comGetChar
*	功能说明: 从接收缓冲区读取1字节，非阻塞。无论有无数据均立即返回。
*	形    参: _ucPort: 端口号(COM1 - COM5)
*			  _pByte: 接收到的数据存放在这个地址
*	返 回 值: 0 表示无数据, 1 表示读取到有效字节
*********************************************************************************************************
*/
uint8_t comGetChar(COM_PORT_E _ucPort, uint8_t *_pByte)
{
	UART_T *pUart;
	pUart = ComToUart(_ucPort);
	if (pUart == 0) return 0;
	
	return UartGetChar(pUart, _pByte);
}

/*
*********************************************************************************************************
*	函 数 名: comClearTxFifo
*	功能说明: 清零串口发送缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearTxFifo(COM_PORT_E _ucPort)
{
	UART_T *pUart;
	pUart = ComToUart(_ucPort);
	if(pUart == 0) return;

	pUart->usTxWrite = 0;
	pUart->usTxRead = 0;
	pUart->usTxCount = 0;
}

/*
*********************************************************************************************************
*	函 数 名: comClearRxFifo
*	功能说明: 清零串口接收缓冲区
*	形    参: _ucPort: 端口号(COM1 - COM6)
*	返 回 值: 无
*********************************************************************************************************
*/
void comClearRxFifo(COM_PORT_E _ucPort)
{
	UART_T *pUart;
	pUart = ComToUart(_ucPort);
	if(pUart == 0) return;

	pUart->usRxWrite = 0;
	pUart->usRxRead = 0;
	pUart->usRxCount = 0;
}

/*
*********************************************************************************************************
*	函 数 名: comSetBaud
*	功能说明: 设置串口的波特率
*	形    参: _ucPort: 端口号(COM1 - COM5)
*			  _BaudRate: 波特率，0-4500000， 最大4.5Mbps
*	返 回 值: 无
*********************************************************************************************************
*/
void comSetBaud(COM_PORT_E _ucPort, uint32_t _BaudRate)
{
	USART_TypeDef* USARTx;
	USARTx = ComToUSARTx(_ucPort);
	if(USARTx == 0) return;
		
	USART_SetBaudRate(USARTx, _BaudRate);
}

/*
*********************************************************************************************************
*	函 数 名: USART_SetBaudRate
*	功能说明: 修改波特率寄存器，不更改其他设置。如果使用 USART_Init函数, 则会修改硬件流控参数和RX,TX配置
*			  根据固件库中 USART_Init函数，将其中配置波特率的部分单独提出来封装为一个函数
*	形    参: USARTx : USART1, USART2, USART3, UART4, UART5
*			  BaudRate : 波特率，取值 0 - 4500000
*	返 回 值: 无
*********************************************************************************************************
*/
void USART_SetBaudRate(USART_TypeDef* USARTx, uint32_t BaudRate)
{
	uint32_t tmpreg = 0x00, apbclock = 0x00;
	uint32_t integerdivider = 0x00;
	uint32_t fractionaldivider = 0x00;
	RCC_ClocksTypeDef RCC_ClocksStatus;

	/* Check the parameters */
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_BAUDRATE(BaudRate));  

	/*---------------------------- USART BRR Configuration -----------------------*/
	/* Configure the USART Baud Rate */
	RCC_GetClocksFreq(&RCC_ClocksStatus);

	if ((USARTx == USART1) || (USARTx == USART6))
		apbclock = RCC_ClocksStatus.PCLK2_Frequency;
	else
		apbclock = RCC_ClocksStatus.PCLK1_Frequency;

	/* Determine the integer part */
	if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
		integerdivider = ((25 * apbclock) / (2 * (BaudRate)));    	/* Integer part computing in case Oversampling mode is 8 Samples */
	else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
		integerdivider = ((25 * apbclock) / (4 * (BaudRate)));    	/* Integer part computing in case Oversampling mode is 16 Samples */
	tmpreg = (integerdivider / 100) << 4;
	
	/* Determine the fractional part */
	fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
	if ((USARTx->CR1 & USART_CR1_OVER8) != 0)
		tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((uint8_t)0x07);
	else /* if ((USARTx->CR1 & USART_CR1_OVER8) == 0) */
		tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((uint8_t)0x0F);

	/* Write to USART BRR register */
	USARTx->BRR = (uint16_t)tmpreg;
}

/* 如果是RS485通信，请按如下格式编写函数， 我们仅举了 USART3作为RS485的例子 */
/*
*********************************************************************************************************
*	函 数 名: RS485_InitTXE
*	功能说明: 配置RS485发送使能口线 TXE
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void RS485_InitTXE(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_RS485_TXEN, ENABLE);	/* 打开GPIO时钟 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		/* 设为输出口 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		/* 设为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	/* 无上拉电阻 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	/* IO口最大速度 */
	GPIO_InitStructure.GPIO_Pin = PIN_RS485_TXEN;
	GPIO_Init(PORT_RS485_TXEN, &GPIO_InitStructure);
	
	RS485_RX_EN();	/* 初始化为接受模式 */
}

void RS485_SetBaud(uint32_t _baud)
{
	comSetBaud(COM3, _baud);
}

void RS485_SendBefor(void)
{
	RS485_TX_EN();	/* 切换RS485收发芯片为发送模式 */
}

void RS485_SendOver(void)
{
	RS485_RX_EN();	/* 切换RS485收发芯片为接收模式 */
}

extern void MODH_ReciveNew(uint8_t _data);
void RS485_ReciveNew(uint8_t _byte)
{

}

void RS485_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen)
{
	comSendBuf(COM3, _ucaBuf, _usLen);
}

void RS485_SendStr(char *_pBuf)
{
	RS485_SendBuf((uint8_t *)_pBuf, strlen(_pBuf));
}

/* 以下部分为串口与上位机通信部分函数 */
void SerialUpload_SetBaud(uint32_t _baud)
{
	#if UART2_FIFO_EN == 1
		comSetBaud(COM2, _baud);
	#elif UART5_FIFO_EN == 1
		comSetBaud(COM5, _baud);
	#endif
}

extern void UploadH_ReciveNew(uint8_t _data);
void SerialUpload_ReciveNew(uint8_t _byte)
{
	UploadH_ReciveNew(_byte);	/* 处理串口2接收到新字节 */
}

void SerialUpload_SendBuf(uint8_t *_ucaBuf, uint16_t _usLen)
{
	#if UART2_FIFO_EN == 1
		comSendBuf(COM2, _ucaBuf, _usLen);
	#elif UART5_FIFO_EN == 1
		comSendBuf(COM5, _ucaBuf, _usLen);
	#endif
}

void SerialUpload_SendStr(char *_pBuf)
{
	SerialUpload_SendBuf((uint8_t *)_pBuf, strlen(_pBuf));
}

/*
*********************************************************************************************************
*	函 数 名: UartVarInit
*	功能说明: 初始化串口相关的变量
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void UartVarInit(void)
{
#if UART1_FIFO_EN == 1
	g_tUart1.uart = USART1;						/* STM32 串口设备 */
	g_tUart1.pTxBuf = g_TxBuf1;					/* 发送缓冲区指针 */
	g_tUart1.pRxBuf = g_RxBuf1;					/* 接收缓冲区指针 */
	g_tUart1.usTxBufSize = UART1_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart1.usRxBufSize = UART1_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart1.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart1.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart1.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart1.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart1.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart1.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart1.SendBefor = 0;						/* 发送数据前的回调函数 */
	g_tUart1.SendOver = 0;						/* 发送完毕后的回调函数 */
	g_tUart1.ReciveNew = SerialUpload_ReciveNew;						/* 接收到新数据后的回调函数 */
#endif
	
#if UART2_FIFO_EN == 1
	g_tUart2.uart = USART2;						/* STM32 串口设备 */
	g_tUart2.pTxBuf = g_TxBuf2;					/* 发送缓冲区指针 */
	g_tUart2.pRxBuf = g_RxBuf2;					/* 接收缓冲区指针 */
	g_tUart2.usTxBufSize = UART2_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart2.usRxBufSize = UART2_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart2.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart2.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart2.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart2.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart2.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart2.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart2.SendBefor = 0;		/* 发送数据前的回调函数 */
	g_tUart2.SendOver = 0;			/* 发送完毕后的回调函数 */
	g_tUart2.ReciveNew = 0;		/* 接收到新数据后的回调函数 */
#endif

#if UART3_FIFO_EN == 1
	g_tUart3.uart = USART3;						/* STM32 串口设备 */
	g_tUart3.pTxBuf = g_TxBuf3;					/* 发送缓冲区指针 */
	g_tUart3.pRxBuf = g_RxBuf3;					/* 接收缓冲区指针 */
	g_tUart3.usTxBufSize = UART3_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart3.usRxBufSize = UART3_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart3.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart3.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart3.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart3.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart3.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart3.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart3.SendBefor = RS485_SendBefor;						/* 发送数据前的回调函数 */
	g_tUart3.SendOver = RS485_SendOver;						/* 发送完毕后的回调函数 */
	g_tUart3.ReciveNew = RS485_ReciveNew;						/* 接收到新数据后的回调函数 */
#endif

#if UART5_FIFO_EN == 1
	g_tUart5.uart = UART5;						/* STM32 串口设备 */
	g_tUart5.pTxBuf = g_TxBuf5;					/* 发送缓冲区指针 */
	g_tUart5.pRxBuf = g_RxBuf5;					/* 接收缓冲区指针 */
	g_tUart5.usTxBufSize = UART5_TX_BUF_SIZE;	/* 发送缓冲区大小 */
	g_tUart5.usRxBufSize = UART5_RX_BUF_SIZE;	/* 接收缓冲区大小 */
	g_tUart5.usTxWrite = 0;						/* 发送FIFO写索引 */
	g_tUart5.usTxRead = 0;						/* 发送FIFO读索引 */
	g_tUart5.usRxWrite = 0;						/* 接收FIFO写索引 */
	g_tUart5.usRxRead = 0;						/* 接收FIFO读索引 */
	g_tUart5.usRxCount = 0;						/* 接收到的新数据个数 */
	g_tUart5.usTxCount = 0;						/* 待发送的数据个数 */
	g_tUart5.SendBefor = 0;		/* 发送数据前的回调函数 */
	g_tUart5.SendOver = 0;			/* 发送完毕后的回调函数 */
	g_tUart5.ReciveNew = SerialUpload_ReciveNew;		/* 接收到新数据后的回调函数 */
#endif
}

/*
*********************************************************************************************************
*	函 数 名: InitHardUart
*	功能说明: 配置串口的硬件参数（波特率，数据位，停止位，起始位，校验位，中断使能）适合于STM32-F4开发板
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
static void InitHardUart(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;

#if UART1_FIFO_EN == 1		/* 串口1 TX = PB6   RX = PB7 */
		/* 第1步： 配置GPIO */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
		/* 配置 USART Tx 为复用功能 */
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* 配置 USART Rx 为复用功能 */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* 第2步： 配置串口硬件参数 */
		USART_InitStructure.USART_BaudRate = UART1_BAUD;	/* 波特率 */
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(USART1, &USART_InitStructure);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
		/*
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			注意: 不要在此处打开发送中断
			发送中断使能在SendUart()函数打开
		*/
		USART_Cmd(USART1, ENABLE);		/* 使能串口 */
		USART_ClearFlag(USART1, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART2_FIFO_EN == 1		/* 串口2 TX = PD5   RX = PD6 */
	/* 第1步： 配置GPIO */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

		/* 配置 USART Tx 为复用功能 */
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* 配置 USART Rx 为复用功能 */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* 第2步： 配置串口硬件参数 */
		USART_InitStructure.USART_BaudRate = UART2_BAUD;	/* 波特率 */
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
		USART_Init(USART2, &USART_InitStructure);

		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
		/*
			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
			注意: 不要在此处打开发送中断
			发送中断使能在SendUart()函数打开
		*/
		USART_Cmd(USART2, ENABLE);		/* 使能串口 */
		USART_ClearFlag(USART2, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART3_FIFO_EN == 1			/* 串口3 TX = PB10   RX = PB11 */
	/* 第1步： 配置GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	/* 配置 USART Tx 为复用功能 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 配置 USART Rx 为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 第2步： 配置串口硬件参数 */
	USART_InitStructure.USART_BaudRate = UART3_BAUD;	/* 波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
	/*
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		注意: 不要在此处打开发送中断
		发送中断使能在SendUart()函数打开
	*/
	USART_Cmd(USART3, ENABLE);		/* 使能串口 */
	USART_ClearFlag(USART3, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif

#if UART5_FIFO_EN == 1		/* 串口5 TX = PC12   RX = PD2 */
	/* 第1步： 配置GPIO */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

		/* 配置 USART Tx 为复用功能 */
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	/* 输出类型为推挽 */
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	/* 内部上拉电阻使能 */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	/* 复用模式 */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		/* 配置 USART Rx 为复用功能 */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOD, &GPIO_InitStructure);

		/* 第2步： 配置串口硬件参数 */
		USART_InitStructure.USART_BaudRate = UART5_BAUD;	/* 波特率 */
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No ;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
		USART_Init(UART5, &USART_InitStructure);

		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
		/*
			USART_ITConfig(UART5, USART_IT_TXE, ENABLE);
			注意: 不要在此处打开发送中断
			发送中断使能在SendUart()函数打开
		*/
		USART_Cmd(UART5, ENABLE);		/* 使能串口 */
		USART_ClearFlag(UART5, USART_FLAG_TC);     /* 清发送完成标志，Transmission Complete flag */
#endif
}

/*
*********************************************************************************************************
*	函 数 名: ConfigUartNVIC
*	功能说明: 配置串口硬件中断.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void ConfigUartNVIC(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#if UART1_FIFO_EN == 1
	/* 使能串口1中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
	
#if UART2_FIFO_EN == 1
	/* 使能串口2中断 */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#if UART3_FIFO_EN == 1
	/* 使能串口3中断t */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif

#if UART5_FIFO_EN == 1
	/* 使能串口5中断 */
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

/*
*********************************************************************************************************
*	函 数 名: UartSend
*	功能说明: 填写数据到UART发送缓冲区,并启动发送中断。中断处理函数发送完毕后，自动关闭发送中断
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
static void UartSend(UART_T *_pUart, uint8_t *_ucaBuf, uint16_t _usLen)
{
	uint16_t i;

	for (i = 0; i < _usLen; i++)
	{
		/* 如果发送缓冲区已经满了，则等待缓冲区有空余 */
		while (1)
		{
			__IO uint16_t usCount;

			DISABLE_INT();
			usCount = _pUart->usTxCount;
			ENABLE_INT();
			if (usCount < _pUart->usTxBufSize) 
				break;
			else
				USART_ITConfig(_pUart->uart, USART_IT_TXE, ENABLE);	//使能发送中断
		}

		/* 将新数据填入发送缓冲区 */
		_pUart->pTxBuf[_pUart->usTxWrite] = _ucaBuf[i];

		DISABLE_INT();
		if (++_pUart->usTxWrite >= _pUart->usTxBufSize)
			_pUart->usTxWrite = 0;
		
		_pUart->usTxCount++;
		ENABLE_INT();
	}

	USART_ITConfig(_pUart->uart, USART_IT_TXE, ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: UartGetChar
*	功能说明: 从串口接收缓冲区读取1字节数据 （用于主程序调用）
*	形    参: _pUart : 串口设备
*			  _pByte : 存放读取数据的指针
*	返 回 值: 0 表示无数据  1表示读取到数据
*********************************************************************************************************
*/
static uint8_t UartGetChar(UART_T *_pUart, uint8_t *_pByte)
{
	uint16_t usCount;

	/* usRxWrite 变量在中断函数中被改写，主程序读取该变量时，必须进行临界区保护 */
	DISABLE_INT();
	usCount = _pUart->usRxCount;
	ENABLE_INT();

	/* 如果读和写索引相同，则返回0 */
	if (usCount == 0)	/* 已经没有数据 */
		return 0;
	
	else
	{
		*_pByte = _pUart->pRxBuf[_pUart->usRxRead];		/* 从串口接收FIFO取1个数据 */

		/* 改写FIFO读索引 */
		DISABLE_INT();
		if (++_pUart->usRxRead >= _pUart->usRxBufSize)
			_pUart->usRxRead = 0;

		_pUart->usRxCount--;
		ENABLE_INT();
		return 1;
	}
}

/*
*********************************************************************************************************
*	函 数 名: UartIRQ
*	功能说明: 供中断服务程序调用，通用串口中断处理函数
*	形    参: _pUart : 串口设备
*	返 回 值: 无
*********************************************************************************************************
*/
static void UartIRQ(UART_T *_pUart)
{
	///////////* 处理接收中断  */
	if (USART_GetITStatus(_pUart->uart, USART_IT_RXNE) != RESET)
	{
		/* 从串口接收数据寄存器读取数据存放到接收FIFO */
		uint8_t ch;

		ch = USART_ReceiveData(_pUart->uart);
		_pUart->pRxBuf[_pUart->usRxWrite] = ch;
		if (++_pUart->usRxWrite >= _pUart->usRxBufSize)
			_pUart->usRxWrite = 0;
		
		if (_pUart->usRxCount < _pUart->usRxBufSize)
			_pUart->usRxCount++;
		
		/* 回调函数,通知应用程序收到新数据,一般是发送1个消息或者设置一个标记 */
		if (_pUart->ReciveNew)
			_pUart->ReciveNew(ch);
	}

	///////////* 处理发送缓冲区空中断 */
	if (USART_GetITStatus(_pUart->uart, USART_IT_TXE) != RESET)
	{
		if (_pUart->usTxCount == 0)
		{
			USART_ITConfig(_pUart->uart, USART_IT_TXE, DISABLE);
			USART_ITConfig(_pUart->uart, USART_IT_TC, ENABLE);
		}
		else
		{
			/* 从发送FIFO取1个字节写入串口发送数据寄存器 */
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
				_pUart->usTxRead = 0;
			
			_pUart->usTxCount--;
		}
	}
	
	///////////* 数据bit位全部发送完毕的中断 */
	else if (USART_GetITStatus(_pUart->uart, USART_IT_TC) != RESET)
	{
		if (_pUart->usTxCount == 0)
		{
			USART_ITConfig(_pUart->uart, USART_IT_TC, DISABLE);
			/* 回调函数,将RS485芯片设置为接收模式，避免抢占总线 */
			if (_pUart->SendOver)
				_pUart->SendOver();
		}
		else	/* 正常情况下，不会进入此分支 */
		{
			USART_SendData(_pUart->uart, _pUart->pTxBuf[_pUart->usTxRead]);
			if (++_pUart->usTxRead >= _pUart->usTxBufSize)
				_pUart->usTxRead = 0;
			
			_pUart->usTxCount--;
		}
	}
}

/*
*********************************************************************************************************
*	函 数 名: USART1_IRQHandler  USART2_IRQHandler USART3_IRQHandler UART4_IRQHandler UART5_IRQHandler
*	功能说明: USART中断服务程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
#if UART1_FIFO_EN == 1
void USART1_IRQHandler(void)
{
	UartIRQ(&g_tUart1);
}
#endif

#if UART2_FIFO_EN == 1
void USART2_IRQHandler(void)
{
	UartIRQ(&g_tUart2);
}
#endif

#if UART3_FIFO_EN == 1
void USART3_IRQHandler(void)
{
	UartIRQ(&g_tUart3);
}
#endif

#if UART5_FIFO_EN == 1
void UART5_IRQHandler(void)
{
	UartIRQ(&g_tUart5);
}
#endif

/*
*********************************************************************************************************
*	函 数 名: 重定义fputc函数用于printf函数打印输出
*	功能说明: USART中断服务程序
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
  bsp_LedOn(2);
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
  bsp_LedOff(2);
	return ch;
}
#endif

