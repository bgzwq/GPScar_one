#include "stm32f10x.h"
#include "NRF24L01.h"

/**
  * NRF24L01驱动（SPI1硬件 + 软件CSN/CE）
  * 引脚：SCK-PA5  MISO-PA6  MOSI-PA7  CSN-PA4  CE-PB0  IRQ-PB1（未用，轮询方式）
  * 模式：发送端（PTX），开启自动应答（ACK），自动重发
  */

/*NRF24L01指令*/
#define NRF_CMD_R_REGISTER		0x00	//读寄存器（低5位为寄存器地址）
#define NRF_CMD_W_REGISTER		0x20	//写寄存器（低5位为寄存器地址）
#define NRF_CMD_R_RX_PAYLOAD	0x61	//读接收数据
#define NRF_CMD_W_TX_PAYLOAD	0xA0	//写发送数据
#define NRF_CMD_FLUSH_TX		0xE1	//清空发送FIFO
#define NRF_CMD_FLUSH_RX		0xE2	//清空接收FIFO
#define NRF_CMD_NOP				0xFF	//空指令（用于读状态寄存器）

/*NRF24L01寄存器地址*/
#define NRF_REG_CONFIG			0x00	//配置寄存器
#define NRF_REG_EN_AA			0x01	//自动应答使能
#define NRF_REG_EN_RXADDR		0x02	//接收通道使能
#define NRF_REG_SETUP_AW		0x03	//地址宽度设置
#define NRF_REG_SETUP_RETR		0x04	//自动重发设置
#define NRF_REG_RF_CH			0x05	//射频频道
#define NRF_REG_RF_SETUP		0x06	//射频参数
#define NRF_REG_STATUS			0x07	//状态寄存器
#define NRF_REG_RX_ADDR_P0		0x0A	//接收通道0地址
#define NRF_REG_TX_ADDR			0x10	//发送地址
#define NRF_REG_RX_PW_P0		0x11	//接收通道0载荷宽度
#define NRF_REG_FIFO_STATUS		0x17	//FIFO状态

/*STATUS寄存器位*/
#define NRF_STATUS_RX_DR		0x40
#define NRF_STATUS_TX_DS		0x20
#define NRF_STATUS_MAX_RT		0x10

/*通信地址（与NRF24L01.h中的宏配置一致）*/
static uint8_t NRF24L01_Address[5] = NRF24L01_ADDR;

/*CSN/CE电平控制宏*/
#define NRF_CSN_LOW()	GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define NRF_CSN_HIGH()	GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define NRF_CE_LOW()	GPIO_ResetBits(GPIOB, GPIO_Pin_0)
#define NRF_CE_HIGH()	GPIO_SetBits(GPIOB, GPIO_Pin_0)

/**
  * 函    数：SPI1收发一个字节
  * 参    数：Byte 要发送的字节
  * 返 回 值：接收到的字节
  */
static uint8_t NRF24L01_SPI_SwapByte(uint8_t Byte)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1, Byte);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SPI1);
}

/**
  * 函    数：向NRF24L01发送一条无参数指令
  */
static void NRF24L01_WriteCmd(uint8_t Cmd)
{
	NRF_CSN_LOW();
	NRF24L01_SPI_SwapByte(Cmd);
	NRF_CSN_HIGH();
}

/**
  * 函    数：写NRF24L01寄存器（单字节）
  */
static void NRF24L01_WriteReg(uint8_t Reg, uint8_t Value)
{
	NRF_CSN_LOW();
	NRF24L01_SPI_SwapByte(NRF_CMD_W_REGISTER | Reg);
	NRF24L01_SPI_SwapByte(Value);
	NRF_CSN_HIGH();
}

/**
  * 函    数：读NRF24L01寄存器（单字节）
  */
static uint8_t NRF24L01_ReadReg(uint8_t Reg)
{
	uint8_t Value;
	NRF_CSN_LOW();
	NRF24L01_SPI_SwapByte(NRF_CMD_R_REGISTER | Reg);
	Value = NRF24L01_SPI_SwapByte(NRF_CMD_NOP);
	NRF_CSN_HIGH();
	return Value;
}

/**
  * 函    数：向NRF24L01发送带数据的指令（如写TX载荷）
  */
static void NRF24L01_WriteCmdBuf(uint8_t Cmd, uint8_t *Buf, uint8_t Length)
{
	uint8_t i;
	NRF_CSN_LOW();
	NRF24L01_SPI_SwapByte(Cmd);
	for (i = 0; i < Length; i ++)
	{
		NRF24L01_SPI_SwapByte(Buf[i]);
	}
	NRF_CSN_HIGH();
}

/**
  * 函    数：向NRF24L01指定寄存器写入多个字节
  */
static void NRF24L01_WriteBuf(uint8_t Reg, uint8_t *Buf, uint8_t Length)
{
	uint8_t i;
	NRF_CSN_LOW();
	NRF24L01_SPI_SwapByte(NRF_CMD_W_REGISTER | Reg);
	for (i = 0; i < Length; i ++)
	{
		NRF24L01_SPI_SwapByte(Buf[i]);
	}
	NRF_CSN_HIGH();
}

/**
  * 函    数：从NRF24L01指定寄存器读出多个字节
  */
static void NRF24L01_ReadBuf(uint8_t Reg, uint8_t *Buf, uint8_t Length)
{
	uint8_t i;
	NRF_CSN_LOW();
	NRF24L01_SPI_SwapByte(NRF_CMD_R_REGISTER | Reg);
	for (i = 0; i < Length; i ++)
	{
		Buf[i] = NRF24L01_SPI_SwapByte(NRF_CMD_NOP);
	}
	NRF_CSN_HIGH();
}

/**
  * 函    数：NRF24L01引脚和SPI1初始化
  */
static void NRF24L01_GPIO_SPI_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*SCK(PA5)、MOSI(PA7)复用推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*MISO(PA6)上拉输入*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*CSN(PA4)推挽输出*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*CE(PB0)推挽输出，IRQ(PB1)上拉输入（备用）*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	NRF_CSN_HIGH();
	NRF_CE_LOW();
	
	/*SPI1配置：模式0，8位，MSB先行，波特率72/16=4.5MHz（NRF24L01上限10MHz）*/
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_Cmd(SPI1, ENABLE);
}

/**
  * 函    数：NRF24L01初始化，配置为发送模式（PTX，带自动ACK）
  * 参    数：无
  * 返 回 值：1：初始化成功（模块在线），0：模块未检测到
  */
uint8_t NRF24L01_Init(void)
{
	NRF24L01_GPIO_SPI_Init();
	
	/*检测模块是否在线，不在线则直接返回失败*/
	if (NRF24L01_Check() == 0)
	{
		return 0;
	}
	
	NRF_CE_LOW();
	
	NRF24L01_WriteReg(NRF_REG_EN_AA, 0x01);				//使能通道0自动应答
	NRF24L01_WriteReg(NRF_REG_EN_RXADDR, 0x01);			//使能接收通道0（用于接收ACK）
	NRF24L01_WriteReg(NRF_REG_SETUP_AW, 0x03);			//地址宽度5字节
	NRF24L01_WriteReg(NRF_REG_SETUP_RETR, 0x5F);		//重发间隔1500us，最多15次
	NRF24L01_WriteReg(NRF_REG_RF_CH, NRF24L01_CHANNEL);	//射频频道
	NRF24L01_WriteReg(NRF_REG_RF_SETUP, 0x0F);			//2Mbps，发射功率0dBm
	NRF24L01_WriteReg(NRF_REG_RX_PW_P0, NRF24L01_PAYLOAD);	//通道0载荷宽度
	
	/*发送地址 = 接收通道0地址（ACK通过通道0返回）*/
	NRF24L01_WriteBuf(NRF_REG_TX_ADDR, NRF24L01_Address, 5);
	NRF24L01_WriteBuf(NRF_REG_RX_ADDR_P0, NRF24L01_Address, 5);
	
	NRF24L01_WriteCmd(NRF_CMD_FLUSH_TX);				//清空发送FIFO
	NRF24L01_WriteCmd(NRF_CMD_FLUSH_RX);				//清空接收FIFO
	NRF24L01_WriteReg(NRF_REG_STATUS, 0x70);			//清除所有中断标志
	
	/*上电，PTX模式，屏蔽所有中断（使用轮询）*/
	NRF24L01_WriteReg(NRF_REG_CONFIG, 0x7E);
	
	return 1;
}

/**
  * 函    数：检测NRF24L01是否在线
  * 参    数：无
  * 返 回 值：1：在线，0：离线
  * 说    明：通过写入并读回发送地址进行校验
  */
uint8_t NRF24L01_Check(void)
{
	uint8_t Buf[5];
	uint8_t i;
	
	NRF24L01_WriteBuf(NRF_REG_TX_ADDR, NRF24L01_Address, 5);
	NRF24L01_ReadBuf(NRF_REG_TX_ADDR, Buf, 5);
	
	for (i = 0; i < 5; i ++)
	{
		if (Buf[i] != NRF24L01_Address[i])
		{
			return 0;
		}
	}
	return 1;
}

/**
  * 函    数：发送一个数据包（等待ACK应答）
  * 参    数：Data 要发送的数据
  * 参    数：Length 数据长度，不超过NRF24L01_PAYLOAD
  * 返 回 值：1：发送成功（收到ACK），0：发送失败（达到最大重发次数）
  */
uint8_t NRF24L01_SendPacket(uint8_t *Data, uint8_t Length)
{
	uint8_t Status;
	uint16_t Timeout;
	
	if (Length > NRF24L01_PAYLOAD) {Length = NRF24L01_PAYLOAD;}
	
	NRF_CE_LOW();
	
	/*清空发送FIFO并写入数据*/
	NRF24L01_WriteCmd(NRF_CMD_FLUSH_TX);
	NRF24L01_WriteCmdBuf(NRF_CMD_W_TX_PAYLOAD, Data, Length);
	
	/*CE拉高至少10us，启动发送*/
	NRF_CE_HIGH();
	
	/*轮询等待发送完成（TX_DS）或达到最大重发次数（MAX_RT）*/
	Timeout = 50000;
	do
	{
		Status = NRF24L01_ReadReg(NRF_REG_STATUS);
		Timeout --;
	} while (!(Status & (NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT)) && Timeout);
	
	NRF_CE_LOW();
	
	if (Status & NRF_STATUS_TX_DS)
	{
		/*发送成功，清除标志位*/
		NRF24L01_WriteReg(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_RX_DR);
		return 1;
	}
	else
	{
		/*达到最大重发次数，清除标志位并清空FIFO*/
		NRF24L01_WriteReg(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
		NRF24L01_WriteCmd(NRF_CMD_FLUSH_TX);
		return 0;
	}
}
