#ifndef __GPS_H
#define __GPS_H

#include "stm32f4xx.h"

/*---------------------- GPS配置宏 (DX-GP10, NMEA0183) ------------------------*/
/*
 * USART6: TX=PG14, RX=PG9, 复用AF8
 * 接收使用 DMA2 Stream2 Channel5 循环模式 + USART空闲中断,
 * 全部为静态缓冲, 不使用动态内存
 */
#define GPS_BaudRate	9600		// DX-GP10默认9600, 如修改过模块波特率请同步修改

#define GPS_USART		USART6
#define GPS_USART_CLK	RCC_APB2Periph_USART6
#define GPS_IRQn		USART6_IRQn

#define GPS_GPIO_CLK	RCC_AHB1Periph_GPIOG
#define GPS_AF			GPIO_AF_USART6

#define GPS_TX_PORT		GPIOG
#define GPS_TX_PIN		GPIO_Pin_14
#define GPS_TX_PINSRC	GPIO_PinSource14

#define GPS_RX_PORT		GPIOG
#define GPS_RX_PIN		GPIO_Pin_9
#define GPS_RX_PINSRC	GPIO_PinSource9

#define GPS_DMA_CLK		RCC_AHB1Periph_DMA2
#define GPS_DMA_STREAM	DMA2_Stream2
#define GPS_DMA_CHANNEL	DMA_Channel_5

#define GPS_DMA_BUF_SIZE	1024	// DMA循环接收缓冲区(静态)
#define GPS_LINE_BUF_SIZE	96		// 单条NMEA语句缓冲(最大82字符)

/* 完整语句队列深度: 1Hz定位一次突发5~8条语句, 主循环100ms消费一次,
 * 队列取10足够容纳两次突发, 不会因为解析不及时丢语句 */
#define GPS_LINE_QUEUE		10

/* GPS_Process() 单次调用最多解析的语句条数 */
#define GPS_PARSE_BURST		4

/*---------------------- 定位数据 ----------------------------*/
typedef struct
{
	uint8_t  valid;			// 定位有效: RMC状态为A(或GGA定位质量>=1兜底)
	uint8_t  fixQuality;	// GGA定位质量: 0无效 1单点 2差分 ...
	uint8_t  satellites;	// 使用卫星数
	float	 hdop;			// 水平精度因子
	float	 latitude;		// 纬度(度), 北纬为正
	float	 longitude;		// 经度(度), 东经为正
	int32_t  latE7;			// 纬度 × 1e7 定点(约1.1cm), 轨迹记录/绘图用
	int32_t  lonE7;			// 经度 × 1e7 定点
	float	 altitude;		// 海拔高度 m
	float	 speedKnots;	// 地面速度 节
	float	 speedKmh;		// 地面速度 km/h
	float	 course;		// 地面航向角 度
	uint8_t  hour, minute, second;	// UTC时间
	uint8_t  day, month;
	uint16_t year;					// UTC日期
	volatile uint8_t updated;		// 1=解析到新数据(GGA或RMC), 读后清零
	volatile uint8_t fixUpdated;	// 1=解析到新的RMC(位置/有效性权威来源), 读后清零
} GPS_DataTypeDef;

/*---------------------- 函数声明 ----------------------------*/

void GPS_Init(void);								// 初始化USART6+DMA接收
uint8_t GPS_Process(void);								// 主循环调用: 接收并解析NMEA语句
GPS_DataTypeDef *GPS_GetData(void);					// 获取定位数据指针
void GPS_SendString(char *str);						// 阻塞发送(可用于下发模块配置命令)
uint16_t GPS_GetQueueDropped(void);					// 因队列满被丢弃的语句条数(诊断用)
uint32_t GPS_GetRmcCount(void);						// 已解析的RMC语句条数(诊断定位是否生效)

#endif //__GPS_H
