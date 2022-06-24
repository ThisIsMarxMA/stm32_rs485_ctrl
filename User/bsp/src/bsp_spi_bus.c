/*
*********************************************************************************************************
*
*	模块名称 : SPI总线驱动
*	文件名称 : bsp_spi_bus.h
*	版    本 : V1.2
*	说    明 : SPI总线底层驱动。提供SPI配置、收发数据、多设备共享SPI支持。
*	修改记录 :
*		版本号  日期        作者    说明
*   V1.0  2014-10-24 armfly  首版。将串行FLASH、TSC2046、VS1053、AD7705、ADS1256等SPI设备的配置
*									和收发数据的函数进行汇总分类。并解决不同速度的设备间的共享问题。
*		V1.1	2015-02-25 armfly  硬件SPI时，没有开启GPIOB时钟，已解决。
*		V1.2	2015-07-23 armfly  修改 bsp_SPI_Init() 函数，增加开关SPI时钟的语句。规范硬件SPI和软件SPI的宏定义.
*
*	Copyright (C), 2015-2016, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"

/*
	
	PB3/SPI3_SCK/SPI1_SCK
	PB4/SPI3_MISO/SPI1_MISO
	PB5/SPI3_MOSI/SPI1_MOSI
	PC2/SPI2_MISO
	PI3/SPI2_MOSI
	PI1/SPI2_SCK
*/
#ifdef HARD_SPI
	#define SPI_HARD	SPI2
	#define RCC_SPI		RCC_APB2Periph_SPI2
	
	/* SPI or I2S mode selection masks */
	#define SPI_Mode_Select      ((uint16_t)0xF7FF)
	#define I2S_Mode_Select      ((uint16_t)0x0800) 
	
	/* SPI registers Masks */
	#define CR1_CLEAR_Mask       ((uint16_t)0x3040)
	#define I2SCFGR_CLEAR_Mask   ((uint16_t)0xF040)

	/* SPI SPE mask */
	#define CR1_SPE_Set          ((uint16_t)0x0040)
	#define CR1_SPE_Reset        ((uint16_t)0xFFBF)
#endif

uint8_t g_spi_busy = 0;		/* SPI 总线共享标志 */

/*
*********************************************************************************************************
*	函 数 名: bsp_InitSPIBus
*	功能说明: 配置SPI总线。 只包括 SCK、 MOSI、 MISO口线的配置。不包括片选CS，也不包括外设芯片特有的INT、BUSY等
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_InitSPIBus(void)
{
#ifdef HARD_SPI
	/* 硬件SPI */
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* 开启GPIO时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOI, ENABLE);

	/* 配置 SCK, MISO 、 MOSI 为复用功能 */
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource3, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOI, GPIO_PinSource1, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource2, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_1;
	GPIO_Init(GPIOI, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* 打开SPI时钟 */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	
	bsp_SPI_Init(	SPI_Direction_2Lines_FullDuplex | 
								SPI_Mode_Master | 
								SPI_DataSize_8b | 
								SPI_CPOL_Low | 
								SPI_CPHA_1Edge | 
								SPI_NSS_Soft | 
								SPI_BaudRatePrescaler_64 | 
								SPI_FirstBit_MSB);	
	
	/* Activate the SPI mode (Reset I2SMOD bit in I2SCFGR register) */
	SPI_HARD->I2SCFGR &= SPI_Mode_Select;		/* 选择SPI模式，不是I2S模式 */

	/*---------------------------- SPIx CRCPOLY Configuration --------------------*/
	/* Write to SPIx CRCPOLY */
	SPI_HARD->CRCPR = 7;		/* 一般不用 */


	SPI_Cmd(SPI_HARD, DISABLE);			/* 先禁止SPI  */

	SPI_Cmd(SPI_HARD, ENABLE);			/* 使能SPI  */
#endif	
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SPI_Init
*	功能说明: 配置STM32内部SPI硬件的工作模式。 简化库函数，提高执行效率。 仅用于SPI接口间切换。
*	形    参: _cr1 寄存器值
*	返 回 值: 无
*********************************************************************************************************
*/
#ifdef HARD_SPI		/* 硬件SPI */
void bsp_SPI_Init(uint16_t _cr1)
{
	SPI_HARD->CR1 = ((SPI_HARD->CR1 & CR1_CLEAR_Mask) | _cr1);
	  
	//SPI_Cmd(SPI_HARD, DISABLE);			/* 先禁止SPI  */	    
    SPI_HARD->CR1 &= CR1_SPE_Reset;	/* Disable the selected SPI peripheral */

	//SPI_Cmd(SPI_HARD, ENABLE);			/* 使能SPI  */		    
    SPI_HARD->CR1 |= CR1_SPE_Set;	  /* Enable the selected SPI peripheral */
}
#endif

/*
*********************************************************************************************************
*	函 数 名: bsp_spiWrite0
*	功能说明: 向SPI总线发送一个字节。SCK上升沿采集数据, SCK空闲时为低电平。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_spiWrite0(uint8_t _ucByte)
{
#ifdef HARD_SPI		/* 硬件SPI */
	/* 等待发送缓冲区空 */
	while(SPI_I2S_GetFlagStatus(SPI_HARD, SPI_I2S_FLAG_TXE) == RESET);

	/* 发送一个字节 */
	SPI_I2S_SendData(SPI_HARD, _ucByte);

	/* 等待数据接收完毕 */
	while(SPI_I2S_GetFlagStatus(SPI_HARD, SPI_I2S_FLAG_RXNE) == RESET);

	/* 读取接收到的数据 */
	SPI_I2S_ReceiveData(SPI_HARD);
#endif
}

/*
*********************************************************************************************************
*	函 数 名: bsp_spiRead0
*	功能说明: 从SPI总线接收8个bit数据。 SCK上升沿采集数据, SCK空闲时为低电平。
*	形    参: 无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t bsp_spiRead0(void)
{
#ifdef HARD_SPI		/* 硬件SPI */
	uint8_t read;

	/* 等待发送缓冲区空 */
	while(SPI_I2S_GetFlagStatus(SPI_HARD, SPI_I2S_FLAG_TXE) == RESET);

	/* 发送一个字节 */
	SPI_I2S_SendData(SPI_HARD, 0);

	/* 等待数据接收完毕 */
	while(SPI_I2S_GetFlagStatus(SPI_HARD, SPI_I2S_FLAG_RXNE) == RESET);

	/* 读取接收到的数据 */
	read = SPI_I2S_ReceiveData(SPI_HARD);

	/* 返回读到的数据 */
	return read;
#endif
}

/*
*********************************************************************************************************
*	函 数 名: bsp_spiWrite1
*	功能说明: 向SPI总线发送一个字节。  SCK上升沿采集数据, SCK空闲时为高电平
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_spiWrite1(uint8_t _ucByte)
{
#ifdef HARD_SPI		/* 硬件SPI */
	/* 等待发送缓冲区空 */
	while(SPI_I2S_GetFlagStatus(SPI_HARD, SPI_I2S_FLAG_TXE) == RESET);

	/* 发送一个字节 */
	SPI_I2S_SendData(SPI_HARD, _ucByte);

	/* 等待数据接收完毕 */
	while(SPI_I2S_GetFlagStatus(SPI_HARD, SPI_I2S_FLAG_RXNE) == RESET);

	/* 读取接收到的数据 */
	SPI_I2S_ReceiveData(SPI_HARD);
#endif
}

/*
*********************************************************************************************************
*	函 数 名: bsp_spiRead1
*	功能说明: 从SPI总线接收8个bit数据。  SCK上升沿采集数据, SCK空闲时为高电平
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
uint8_t bsp_spiRead1(void)
{
#ifdef HARD_SPI		/* 硬件SPI */
	uint8_t read;

	/* 等待发送缓冲区空 */
	while(SPI_I2S_GetFlagStatus(SPI_HARD, SPI_I2S_FLAG_TXE) == RESET);

	/* 发送一个字节 */
	SPI_I2S_SendData(SPI_HARD, 0);

	/* 等待数据接收完毕 */
	while(SPI_I2S_GetFlagStatus(SPI_HARD, SPI_I2S_FLAG_RXNE) == RESET);

	/* 读取接收到的数据 */
	read = SPI_I2S_ReceiveData(SPI_HARD);

	/* 返回读到的数据 */
	return read;
#endif
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SpiBusEnter
*	功能说明: 占用SPI总线
*	形    参: 无
*	返 回 值: 0 表示不忙  1表示忙
*********************************************************************************************************
*/
void bsp_SpiBusEnter(void)
{
	g_spi_busy = 1;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SpiBusExit
*	功能说明: 释放占用的SPI总线
*	形    参: 无
*	返 回 值: 0 表示不忙  1表示忙
*********************************************************************************************************
*/
void bsp_SpiBusExit(void)
{
	g_spi_busy = 0;
}

/*
*********************************************************************************************************
*	函 数 名: bsp_SpiBusBusy
*	功能说明: 判断SPI总线忙。方法是检测其他SPI芯片的片选信号是否为1
*	形    参: 无
*	返 回 值: 0 表示不忙  1表示忙
*********************************************************************************************************
*/
uint8_t bsp_SpiBusBusy(void)
{
	return g_spi_busy;
}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
