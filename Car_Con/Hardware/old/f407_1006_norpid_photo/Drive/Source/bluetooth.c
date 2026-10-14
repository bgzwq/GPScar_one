/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   蓝牙/WiFi 串口接口 (USART2)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. USART2: TX=PD5, RX=PD6, 中断接收
	*  2. 接收同时提供两种读取方式:
	*     - BT_ReadByte(): 逐字节读取, 适合单字符遥控指令(如 'F'前进)
	*     - BT_ReadLine(): 按行读取(以 \n 或 \r 结尾), 适合AT指令/字符串指令
	*  3. 发送为阻塞式, 数据量小无需DMA
	*
	************************************************************************************************
***/

#include "bluetooth.h"

/* 字节FIFO */
#define BT_FIFO_SIZE	64
static volatile uint8_t  s_fifo[BT_FIFO_SIZE];
static volatile uint16_t s_fifoHead = 0;	// 写指针(中断)
static volatile uint16_t s_fifoTail = 0;	// 读指针(主循环)

/* 行缓冲 */
static volatile char    s_lineBuf[BT_LINE_BUF_SIZE];
static volatile uint8_t s_lineLen  = 0;
static volatile uint8_t s_lineReady = 0;

/*************************************************************************************************
*	函 数 名:	BT_Init
*
*	函数功能:	初始化 USART2 (PD5/PD6) 并开启接收中断
*
*************************************************************************************************/
void BT_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(BT_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(BT_USART_CLK, ENABLE);

	/* IO复用 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = BT_TX_PIN | BT_RX_PIN;
	GPIO_Init(BT_TX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(BT_TX_PORT, BT_TX_PinSource, BT_AF);
	GPIO_PinAFConfig(BT_RX_PORT, BT_RX_PinSource, BT_AF);

	/* 串口参数: 8N1, 无流控 */
	USART_InitStructure.USART_BaudRate   = BT_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;
	USART_InitStructure.USART_Parity     = USART_Parity_No;
	USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(BT_USART, &USART_InitStructure);

	/* 接收中断 */
	USART_ITConfig(BT_USART, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = BT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(BT_USART, ENABLE);
}

/*************************************************************************************************
*	函 数 名:	USART2_IRQHandler
*
*	函数功能:	接收中断, 字节入FIFO, 同时按行组帧
*
*************************************************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t byte;
	uint16_t next;

	if (USART_GetITStatus(BT_USART, USART_IT_RXNE) != RESET)
	{
		byte = (uint8_t)USART_ReceiveData(BT_USART);

		/* 字节FIFO (溢出则丢弃最旧策略: 直接不写) */
		next = (uint16_t)((s_fifoHead + 1) % BT_FIFO_SIZE);
		if (next != s_fifoTail)
		{
			s_fifo[s_fifoHead] = byte;
			s_fifoHead = next;
		}

		/* 行组帧: \r 或 \n 结束一行 */
		if (!s_lineReady)
		{
			if (byte == '\r' || byte == '\n')
			{
				if (s_lineLen > 0)
				{
					s_lineBuf[s_lineLen] = '\0';
					s_lineReady = 1;
				}
			}
			else if (s_lineLen < BT_LINE_BUF_SIZE - 1)
			{
				s_lineBuf[s_lineLen++] = (char)byte;
			}
			else
			{
				s_lineLen = 0;	// 行超长, 丢弃
			}
		}
	}
}

/*************************************************************************************************
*	函 数 名:	BT_ReadByte
*
*	入口参数:	byte - 接收字节指针
*
*	返 回 值:	1 - 读到数据, 0 - 无数据
*
*************************************************************************************************/
uint8_t BT_ReadByte(uint8_t *byte)
{
	if (s_fifoTail == s_fifoHead) return 0;

	*byte = s_fifo[s_fifoTail];
	s_fifoTail = (uint16_t)((s_fifoTail + 1) % BT_FIFO_SIZE);
	return 1;
}

/*************************************************************************************************
*	函 数 名:	BT_ReadLine
*
*	入口参数:	buf    - 字符串缓冲区
*				maxLen - 缓冲区长度
*
*	返 回 值:	1 - 读到完整一行, 0 - 无新行
*
*************************************************************************************************/
uint8_t BT_ReadLine(char *buf, uint8_t maxLen)
{
	uint8_t i;

	if (!s_lineReady) return 0;

	for (i = 0; i < s_lineLen && i < maxLen - 1; i++)
	{
		buf[i] = s_lineBuf[i];
	}
	buf[i] = '\0';

	s_lineLen   = 0;
	s_lineReady = 0;
	return 1;
}

/*************************************************************************************************
*	函 数 名:	BT_SendByte / BT_SendString / BT_SendData
*
*	函数功能:	阻塞式发送
*
*************************************************************************************************/
void BT_SendByte(uint8_t byte)
{
	USART_SendData(BT_USART, byte);
	while (USART_GetFlagStatus(BT_USART, USART_FLAG_TXE) == RESET);
}

void BT_SendString(char *str)
{
	while (*str)
	{
		BT_SendByte((uint8_t)*str++);
	}
}

void BT_SendData(uint8_t *data, uint16_t len)
{
	while (len--)
	{
		BT_SendByte(*data++);
	}
}
