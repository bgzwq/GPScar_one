/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   ASRPRO 语音识别模块接口 (USART3)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. USART3: TX=PD8, RX=PD9, 中断接收
	*  2. ASRPRO 识别到语音后通过串口发出结果(具体协议在ASRPRO上位机中自定义),
	*     本驱动按行(以 \n 或 \r 结尾)缓存结果, 主循环用 ASR_ReadLine() 取出匹配
	*  3. 同时也支持逐字节读取, 适配自定义二进制协议
	*
	************************************************************************************************
***/

#include "asrpro.h"

/* 字节FIFO */
#define ASR_FIFO_SIZE	64
static volatile uint8_t  s_fifo[ASR_FIFO_SIZE];
static volatile uint16_t s_fifoHead = 0;
static volatile uint16_t s_fifoTail = 0;

/* 行缓冲 */
static volatile char    s_lineBuf[ASR_LINE_BUF_SIZE];
static volatile uint8_t s_lineLen   = 0;
static volatile uint8_t s_lineReady = 0;

/*************************************************************************************************
*	函 数 名:	ASR_Init
*
*	函数功能:	初始化 USART3 (PD8/PD9) 并开启接收中断
*
*************************************************************************************************/
void ASR_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(ASR_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(ASR_USART_CLK, ENABLE);

	/* IO复用 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = ASR_TX_PIN | ASR_RX_PIN;
	GPIO_Init(ASR_TX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(ASR_TX_PORT, ASR_TX_PinSource, ASR_AF);
	GPIO_PinAFConfig(ASR_RX_PORT, ASR_RX_PinSource, ASR_AF);

	/* 串口参数: 8N1, 无流控 */
	USART_InitStructure.USART_BaudRate   = ASR_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;
	USART_InitStructure.USART_Parity     = USART_Parity_No;
	USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(ASR_USART, &USART_InitStructure);

	/* 接收中断 */
	USART_ITConfig(ASR_USART, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = ASR_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(ASR_USART, ENABLE);
}

/*************************************************************************************************
*	函 数 名:	USART3_IRQHandler
*
*	函数功能:	接收中断, 字节入FIFO, 同时按行组帧
*
*************************************************************************************************/
void USART3_IRQHandler(void)
{
	uint8_t byte;
	uint16_t next;

	if (USART_GetITStatus(ASR_USART, USART_IT_RXNE) != RESET)
	{
		byte = (uint8_t)USART_ReceiveData(ASR_USART);

		next = (uint16_t)((s_fifoHead + 1) % ASR_FIFO_SIZE);
		if (next != s_fifoTail)
		{
			s_fifo[s_fifoHead] = byte;
			s_fifoHead = next;
		}

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
			else if (s_lineLen < ASR_LINE_BUF_SIZE - 1)
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
*	函 数 名:	ASR_ReadByte
*
*	返 回 值:	1 - 读到数据, 0 - 无数据
*
*************************************************************************************************/
uint8_t ASR_ReadByte(uint8_t *byte)
{
	if (s_fifoTail == s_fifoHead) return 0;

	*byte = s_fifo[s_fifoTail];
	s_fifoTail = (uint16_t)((s_fifoTail + 1) % ASR_FIFO_SIZE);
	return 1;
}

/*************************************************************************************************
*	函 数 名:	ASR_ReadLine
*
*	入口参数:	buf    - 字符串缓冲区
*				maxLen - 缓冲区长度
*
*	返 回 值:	1 - 读到完整一行, 0 - 无新行
*
*************************************************************************************************/
uint8_t ASR_ReadLine(char *buf, uint8_t maxLen)
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
*	函 数 名:	ASR_SendByte / ASR_SendString
*
*	函数功能:	阻塞式发送
*
*************************************************************************************************/
void ASR_SendByte(uint8_t byte)
{
	USART_SendData(ASR_USART, byte);
	while (USART_GetFlagStatus(ASR_USART, USART_FLAG_TXE) == RESET);
}

void ASR_SendString(char *str)
{
	while (*str)
	{
		ASR_SendByte((uint8_t)*str++);
	}
}
