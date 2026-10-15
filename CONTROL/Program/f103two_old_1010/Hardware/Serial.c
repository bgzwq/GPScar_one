#include "stm32f10x.h"
#include "Serial.h"

/**
  * 蓝牙串口模块（HC-05/HC-06串口透传）
  * USART1：TX-PA9，RX-PA10，9600 8N1（模块默认透传波特率）
  * 配对检测：PA11接蓝牙模块STATE脚，配对连接后输出高电平
  * 接收：RXNE中断 + 环形缓冲区
  * 发送：轮询方式
  */

static uint8_t Serial_RxBuf[SERIAL_RX_BUF_SIZE];	//接收环形缓冲区
static volatile uint8_t Serial_RxHead;				//写指针（中断中更新）
static volatile uint8_t Serial_RxTail;				//读指针（主循环中更新）

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*TX(PA9)复用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*RX(PA10)上拉输入*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*蓝牙STATE脚(PA11)下拉输入：配对连接后模块输出高电平*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*USART1配置：9600 8N1*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);
	
	/*使能接收中断*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/**
  * 函    数：串口发送一个数据包
  * 参    数：Data 要发送的数据
  * 参    数：Length 数据长度
  * 返 回 值：无
  */
void Serial_SendPacket(uint8_t *Data, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Data[i]);
	}
}

/*---------------------- 蓝牙遥控协议(4字节帧) ------------------*/
/*
 * 帧头: 0xA5
 * X轴(0~255): 0x00=左极限, 0xFF=右极限
 * Y轴(0~255): 0x00=上极限(前进), 0xFF=下极限(后退)
 * 按键: 0=未按下, 1=PB12(急停), 2=PB13, 3=PB14, 4=PB15
 *
 * 蓝牙模块连接上后持续发送遥控帧即进入蓝牙控制(手动模式下生效);
 * 超过 LANYA_TIMEOUT_MS 没有新帧则自动停车(掉线保护)
 */

/**
  * 函    数：蓝牙发送一帧遥控数据（4字节帧）
  * 参    数：X 摇杆X轴（0~255，0x00=左极限，0xFF=右极限）
  * 参    数：Y 摇杆Y轴（0~255，0x00=上极限/前进，0xFF=下极限/后退）
  * 参    数：Key 按键数据（0=未按下，1=PB12/急停，2=PB13，3=PB14，4=PB15）
  * 返 回 值：无
  */
void Serial_SendRemoteFrame(uint8_t X, uint8_t Y, uint8_t Key)
{
	uint8_t Frame[4];
	Frame[0] = 0xA5;							//帧头
	Frame[1] = X;								//X轴
	Frame[2] = Y;								//Y轴
	Frame[3] = Key;								//按键数据
	Serial_SendPacket(Frame, 4);
}

/**
  * 函    数：查询蓝牙配对连接状态
  * 参    数：无
  * 返 回 值：1=已配对连接（STATE脚高电平），0=未连接
  */
uint8_t Serial_BTConnected(void)
{
	return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
}

/**
  * 函    数：查询接收缓冲区中可读的字节数
  * 参    数：无
  * 返 回 值：可读字节数
  */
uint8_t Serial_RxAvailable(void)
{
	return (Serial_RxHead - Serial_RxTail) % SERIAL_RX_BUF_SIZE;
}

/**
  * 函    数：从接收缓冲区读取一个字节
  * 参    数：无
  * 返 回 值：读取到的字节（缓冲区空时返回0，需先用Serial_RxAvailable判断）
  */
uint8_t Serial_RxRead(void)
{
	uint8_t Byte = 0;
	if (Serial_RxAvailable())
	{
		Byte = Serial_RxBuf[Serial_RxTail];
		Serial_RxTail = (Serial_RxTail + 1) % SERIAL_RX_BUF_SIZE;
	}
	return Byte;
}

/**
  * 函    数：USART1中断服务函数，接收数据存入环形缓冲区
  * 参    数：无
  * 返 回 值：无
  */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t Next = (Serial_RxHead + 1) % SERIAL_RX_BUF_SIZE;
		uint8_t Data = USART_ReceiveData(USART1);
		if (Next != Serial_RxTail)		//缓冲区未满才写入，满则丢弃新数据
		{
			Serial_RxBuf[Serial_RxHead] = Data;
			Serial_RxHead = Next;
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
