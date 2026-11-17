#include "lidar.h"
#include "timer.h"
#include <string.h>

static uint8_t s_dma[LIDAR_DMA_SIZE];
static uint16_t s_read;
static Lidar_DataTypeDef s_data;

static const uint8_t s_crcTable[256] = {
0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
0xa0,0xed,0x3a,0x77,0xd9,0x94,0x43,0x0e,0x52,0x1f,0xc8,0x85,0x2b,0x66,0xb1,0xfc,
0x5a,0x17,0xc0,0x8d,0x23,0x6e,0xb9,0xf4,0xa8,0xe5,0x32,0x7f,0xd1,0x9c,0x4b,0x06,
0xfa,0xb7,0x60,0x2d,0x83,0xce,0x19,0x54,0x08,0x45,0x92,0xdf,0x71,0x3c,0xeb,0xa6,
0xb4,0xf9,0x2e,0x63,0xcd,0x80,0x57,0x1a,0x46,0x0b,0xdc,0x91,0x3f,0x72,0xa5,0xe8,
0x14,0x59,0x8e,0xc3,0x6d,0x20,0xf7,0xba,0xe6,0xab,0x7c,0x31,0x9f,0xd2,0x05,0x48,
0xee,0xa3,0x74,0x39,0x97,0xda,0x0d,0x40,0x1c,0x51,0x86,0xcb,0x65,0x28,0xff,0xb2,
0x4e,0x03,0xd4,0x99,0x37,0x7a,0xad,0xe0,0xbc,0xf1,0x26,0x6b,0xc5,0x88,0x5f,0x12,
0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
0x9e,0xd3,0x04,0x49,0xe7,0xaa,0x7d,0x30,0x6c,0x21,0xf6,0xbb,0x15,0x58,0x8f,0xc2,
0x64,0x29,0xfe,0xb3,0x1d,0x50,0x87,0xca,0x96,0xdb,0x0c,0x41,0xef,0xa2,0x75,0x38,
0xc4,0x89,0x5e,0x13,0xbd,0xf0,0x27,0x6a,0x36,0x7b,0xac,0xe1,0x4f,0x02,0xd5,0x98,
0x8a,0xc7,0x10,0x5d,0xf3,0xbe,0x69,0x24,0x78,0x35,0xe2,0xaf,0x01,0x4c,0x9b,0xd6,
0x2a,0x67,0xb0,0xfd,0x53,0x1e,0xc9,0x84,0xd8,0x95,0x42,0x0f,0xa1,0xec,0x3b,0x76,
0xd0,0x9d,0x4a,0x07,0xa9,0xe4,0x33,0x7e,0x22,0x6f,0xb8,0xf5,0x5b,0x16,0xc1,0x8c,
0x70,0x3d,0xea,0xa7,0x09,0x44,0x93,0xde,0x82,0xcf,0x18,0x55,0xfb,0xb6,0x61,0x2c
};

void Lidar_Init(void)
{
    GPIO_InitTypeDef gpio;
    USART_InitTypeDef uart;
    DMA_InitTypeDef dma;
    NVIC_InitTypeDef nvic;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOD, &gpio);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
    uart.USART_BaudRate = 230400;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &uart);
    DMA_DeInit(DMA1_Stream1);
    while (DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    dma.DMA_Memory0BaseAddr = (uint32_t)s_dma;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = LIDAR_DMA_SIZE;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &dma);
    DMA_Cmd(DMA1_Stream1, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 5;
    nvic.NVIC_IRQChannelSubPriority = 2;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    USART_Cmd(USART3, ENABLE);
}

static void Lidar_Byte(uint8_t value, uint32_t now)
{
    static uint8_t frame[47];
    static uint8_t length;
    static uint8_t expected;
    uint8_t i, crc = 0;
    uint16_t points, start, end;
    if (length == 0)
    {
        if (value == 0x54) { frame[length++] = value; expected = 0; }
        return;
    }
    frame[length++] = value;
    if (length == 2)
    {
        points = value & 0x1F;
        if (points == 0 || points > 12) { length = 0; return; }
        expected = (uint8_t)(6 + points * 3 + 5);
    }
    if (length < expected) return;
    for (i = 0; i < expected - 1; i++) crc = s_crcTable[crc ^ frame[i]];
    if (crc != frame[expected - 1]) { s_data.crcErrors++; length = 0; return; }
    points = frame[1] & 0x1F;
    start = frame[4] | ((uint16_t)frame[5] << 8);
    end = frame[expected - 5] | ((uint16_t)frame[expected - 4] << 8);
    if (end < start) end += 36000;
    for (i = 0; i < points; i++)
    {
        uint16_t angle = (uint16_t)(start + (uint32_t)(end - start) * i / points);
        uint16_t distance = frame[6 + i * 3] | ((uint16_t)frame[7 + i * 3] << 8);
        uint16_t sector = (angle % 36000) / 100;
        if (distance > 0 && (s_data.distanceMm[sector] == 0 || distance < s_data.distanceMm[sector]))
            s_data.distanceMm[sector] = distance;
    }
    s_data.speed = frame[2] | ((uint16_t)frame[3] << 8);
    s_data.packets++;
    s_data.lastPacketMs = now;
    s_data.online = 1;
    length = 0;
}

void Lidar_Process(uint32_t now)
{
    uint16_t write = (uint16_t)(LIDAR_DMA_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1));
    while (s_read != write)
    {
        Lidar_Byte(s_dma[s_read], now);
        s_read = (s_read + 1) % LIDAR_DMA_SIZE;
    }
    if ((uint32_t)(now - s_data.lastPacketMs) > 200)
    {
        s_data.online = 0;
        memset(s_data.distanceMm, 0, sizeof(s_data.distanceMm));
    }
}

Lidar_DataTypeDef *Lidar_GetData(void) { return &s_data; }

uint16_t Lidar_GetMinDistance(uint16_t center, uint16_t halfWidth)
{
    int i;
    uint16_t min = 0;
    for (i = (int)center - halfWidth; i <= (int)center + halfWidth; i++)
    {
        uint16_t d = s_data.distanceMm[(i + LIDAR_SECTORS) % LIDAR_SECTORS];
        if (d && (min == 0 || d < min)) min = d;
    }
    return min;
}
/*************************************************************************************************
*	函 数 名:	USART3_IRQHandler
*
*	函数功能:	LD19 雷达的空闲中断: 只负责把 IDLE 标志清掉
*
*	说    明:	【这是一个必须存在的中断函数】
*				Lidar_Init() 里使能了 USART3 的 IDLE 中断和 NVIC, 但工程里原来
*				只有 asrpro.c 定义过 USART3_IRQHandler, 而 asrpro.c 并没有加进
*				Keil 工程 -> 向量表里 USART3 落在启动文件的 [WEAK] Default_Handler
*				(B . 死循环)。只要雷达开始吐数据, 总线上一定出现空闲 -> IDLE 置位
*				-> 进死循环, 而且 IDLE 没人清, 中断会一直挂着 -> 整车像卡死。
*				清 IDLE 的顺序必须是"先读 SR 再读 DR", 这里 USART_GetITStatus()
*				读 SR、USART_ReceiveData() 读 DR, 正好满足要求(与 gps.c 同款写法)。
*				真正的数据搬运仍然由 Lidar_Process() 从 DMA 缓冲里做。
*
*************************************************************************************************/
void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(USART3);		/* 读 SR + DR, 清除 IDLE 标志(不能省) */
	}
}
