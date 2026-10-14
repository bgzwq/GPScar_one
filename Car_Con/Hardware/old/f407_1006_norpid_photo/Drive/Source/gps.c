/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   DX-GP10 定位模块驱动 (NMEA0183协议)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. USART6: TX=PG14, RX=PG9, 波特率9600 (可在gps.h中修改GPS_BaudRate)
	*  2. 接收: DMA2 Stream2 Channel5 循环模式 + USART空闲中断,
	*     中断中把DMA数据组帧为NMEA语句, 主循环调用GPS_Process()完成解析,
	*     全程静态缓冲, 不使用动态内存
	*  3. 解析语句: xxGGA (定位/卫星数/海拔) 和 xxRMC (速度/航向/日期),
	*     自动兼容 GP/GL/BD/GN 等任意Talker ID, 校验和不通过的语句直接丢弃
	*  4. 解析结果存入 GPS_GetData() 返回的结构体, updated=1 表示有新数据
	*
	************************************************************************************************
***/

#include "gps.h"

/*---------------- 静态缓冲 (无动态内存) ----------------*/

static uint8_t s_dmaBuf[GPS_DMA_BUF_SIZE];			// DMA循环接收缓冲
static volatile uint16_t s_dmaLastPos = 0;			// 上次消费位置

static char s_lineBuf[GPS_LINE_BUF_SIZE];			// 语句组帧缓冲(中断写)
static volatile uint8_t  s_lineLen = 0;

/* 完整语句队列(环形, 静态, 无动态内存)
 *
 * 为什么要队列:
 *   定位模块一般是 1Hz 输出, 一次突发里连续吐出 GGA/GSA/GSV/RMC/VTG 共5~8条
 *   语句(9600bps 下约 0.4~0.6s)。而主循环 100ms 才调用一次 GPS_Process()。
 *   原来的实现只有一个 s_msgReady 缓冲: 突发中第一条语句收完后 s_msgReady=1,
 *   后面所有字节都被 "if (s_msgReady) continue;" 直接丢掉, 直到主循环把这一条
 *   解析完。也就是说每次突发只能解析出 1 条语句, 而 valid 只在 RMC 里置1,
 *   一旦 RMC 被丢掉, gps->valid 永远是 0 -> 融合定位永远进不了 GPS 模式。
 *   改成队列后整个突发全部入队, 主循环每次批量解析, 不再丢语句。 */
static char s_lineQ[GPS_LINE_QUEUE][GPS_LINE_BUF_SIZE];	// 已收完整的语句
static volatile uint8_t s_qWr = 0;						// 入队指针(仅中断里写)
static volatile uint8_t s_qRd = 0;						// 出队指针(仅主循环写)
static volatile uint16_t s_qDropped = 0;				// 队列满而丢弃的语句数

static GPS_DataTypeDef s_gpsData;					// 定位数据
static uint32_t s_rmcCount = 0;						// 已解析的RMC语句数(诊断GPS是否真的在定位)

/*---------------- DMA接收 ----------------*/

/* 消费DMA缓冲区中新到的字节, 逐字节组帧 */
static void GPS_DmaDrain(void)
{
	uint16_t pos = (uint16_t)(GPS_DMA_BUF_SIZE - DMA_GetCurrDataCounter(GPS_DMA_STREAM));
	uint8_t  byte;

	while (s_dmaLastPos != pos)
	{
		byte = s_dmaBuf[s_dmaLastPos];
		s_dmaLastPos = (uint16_t)((s_dmaLastPos + 1) % GPS_DMA_BUF_SIZE);

		if (byte == '$')
		{
			s_lineLen = 0;
			s_lineBuf[s_lineLen++] = '$';
		}
		else if (byte == '\r' || byte == '\n')
		{
			if (s_lineLen > 1)						// 一条语句结束
			{
				uint8_t i;
				uint8_t next;

				s_lineBuf[s_lineLen] = '\0';

				/* 入队(队列满则丢弃并计数, 不覆盖未解析的数据) */
				next = (uint8_t)((s_qWr + 1) % GPS_LINE_QUEUE);
				if (next != s_qRd)
				{
					for (i = 0; i <= s_lineLen; i++) s_lineQ[s_qWr][i] = s_lineBuf[i];
					s_qWr = next;
				}
				else
				{
					s_qDropped++;
				}
				s_lineLen = 0;
			}
		}
		else if (s_lineLen > 0)
		{
			if (s_lineLen < GPS_LINE_BUF_SIZE - 1) s_lineBuf[s_lineLen++] = (char)byte;
			else s_lineLen = 0;						// 超长丢弃
		}
	}
}

/*************************************************************************************************
*	函 数 名:	USART6_IRQHandler
*
*	函数功能:	空闲中断, 一批NMEA数据到达后触发, 消费DMA数据
*
*************************************************************************************************/
void USART6_IRQHandler(void)
{
	if (USART_GetITStatus(GPS_USART, USART_IT_IDLE) != RESET)
	{
		USART_ReceiveData(GPS_USART);	// 读SR+DR清除IDLE标志
		GPS_DmaDrain();
	}
}

/*---------------- NMEA解析 ----------------*/

/* 取一个字段(逗号或*分隔), 原地截断, p推进到下一字段 */
static char *GPS_Field(char **p)
{
	char *start = *p;
	while (**p && **p != ',' && **p != '*') (*p)++;
	if (**p) { **p = '\0'; (*p)++; }
	return start;
}

static int32_t GPS_ParseInt(const char *s)
{
	int32_t v = 0;
	while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
	return v;
}

/* 用double解析: NMEA 的纬经度是 ddmm.mmmmm, 转成十进制度后
 * 113.103889 这种值用 float 存储只有约 0.8m 的分辨率(2^-17 度),
 * 会让地图上的轨迹出现明显台阶, 所以这里全程用 double 中间量。 */
static double GPS_ParseDouble(const char *s)
{
	double v = 0.0, frac = 0.1;
	uint8_t neg = 0;

	if (*s == '-') { neg = 1; s++; }
	while (*s >= '0' && *s <= '9') { v = v * 10.0 + (*s - '0'); s++; }
	if (*s == '.')
	{
		s++;
		while (*s >= '0' && *s <= '9') { v += (*s - '0') * frac; frac *= 0.1; s++; }
	}
	return neg ? -v : v;
}

/* NMEA格式 ddmm.mmmmm 转十进制度 */
static double GPS_NmeaToDeg(double v)
{
	int32_t deg = (int32_t)(v / 100.0);
	return (double)deg + (v - (double)deg * 100.0) / 60.0;
}

/* 十进制度 -> 1e-7度定点(约1.1cm分辨率), 用于轨迹记录, 不受float精度限制 */
static int32_t GPS_DegToE7(double deg)
{
	return (int32_t)(deg * 10000000.0 + (deg >= 0.0 ? 0.5 : -0.5));
}

static uint8_t GPS_Hex(char c)
{
	if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
	if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
	if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
	return 0;
}

/* 校验和: '$'与'*'之间所有字符异或, 与*后两位十六进制比较 */
static uint8_t GPS_CheckSumOk(char *s)
{
	uint8_t cs = 0;
	char *p = s + 1;	// 跳过'$'

	while (*p && *p != '*') cs ^= (uint8_t)*p++;
	if (*p != '*') return 0;

	return (cs == (uint8_t)((GPS_Hex(p[1]) << 4) | GPS_Hex(p[2])));
}

/* 解析UTC时间 hhmmss.sss */
static void GPS_ParseTime(const char *s)
{
	if (s[0] == '\0') return;
	s_gpsData.hour   = (uint8_t)((s[0] - '0') * 10 + (s[1] - '0'));
	s_gpsData.minute = (uint8_t)((s[2] - '0') * 10 + (s[3] - '0'));
	s_gpsData.second = (uint8_t)((s[4] - '0') * 10 + (s[5] - '0'));
}

/* $--GGA,time,lat,NS,lon,EW,fix,numSv,HDOP,msl,M,... */
static void GPS_ParseGGA(char *s)
{
	char *p = s;
	char *f;

	GPS_Field(&p);					// 语句ID
	GPS_ParseTime(GPS_Field(&p));	// UTC时间

	f = GPS_Field(&p);
	if (f[0] != '\0')
	{
		double lat = GPS_NmeaToDeg(GPS_ParseDouble(f));
		if (GPS_Field(&p)[0] == 'S') lat = -lat;	// 南纬为负
		s_gpsData.latitude = (float)lat;
		s_gpsData.latE7    = GPS_DegToE7(lat);
	}
	else GPS_Field(&p);

	f = GPS_Field(&p);
	if (f[0] != '\0')
	{
		double lon = GPS_NmeaToDeg(GPS_ParseDouble(f));
		if (GPS_Field(&p)[0] == 'W') lon = -lon;	// 西经为负
		s_gpsData.longitude = (float)lon;
		s_gpsData.lonE7     = GPS_DegToE7(lon);
	}
	else GPS_Field(&p);

	s_gpsData.fixQuality = (uint8_t)GPS_ParseInt(GPS_Field(&p));
	s_gpsData.satellites = (uint8_t)GPS_ParseInt(GPS_Field(&p));
	s_gpsData.hdop       = (float)GPS_ParseDouble(GPS_Field(&p));

	/* 高度: 定位无效时该字段为空, 之前直接解析空串会把高度刷成0,
	 * 显示上就是"高度一直掉"; 空字段时保持上一次的有效高度不变 */
	f = GPS_Field(&p);
	if (f[0] != '\0') s_gpsData.altitude = (float)GPS_ParseDouble(f);

	/* 兜底: 有些模块默认不开RMC输出, 若一直收不到RMC语句,
	 * 这里用GGA的定位质量位也把 valid 置起来, 避免GPS永远"无效"。
	 * RMC 与 GGA 在同一突发里, 正常情况两者结论一致。 */
	if (s_gpsData.fixQuality >= 1) s_gpsData.valid = 1;
	else if (s_gpsData.satellites == 0) s_gpsData.valid = 0;
}

/* $--RMC,time,status,lat,NS,lon,EW,spd,cog,date,... */
static void GPS_ParseRMC(char *s)
{
	char *p = s;
	char *f;
	uint32_t date;

	GPS_Field(&p);					// 语句ID
	GPS_ParseTime(GPS_Field(&p));	// UTC时间

	f = GPS_Field(&p);				// 定位状态 A有效 V无效
	s_gpsData.valid = (f[0] == 'A') ? 1 : 0;

	f = GPS_Field(&p);
	if (f[0] != '\0')
	{
		double lat = GPS_NmeaToDeg(GPS_ParseDouble(f));
		if (GPS_Field(&p)[0] == 'S') lat = -lat;
		s_gpsData.latitude = (float)lat;
		s_gpsData.latE7    = GPS_DegToE7(lat);
	}
	else GPS_Field(&p);

	f = GPS_Field(&p);
	if (f[0] != '\0')
	{
		double lon = GPS_NmeaToDeg(GPS_ParseDouble(f));
		if (GPS_Field(&p)[0] == 'W') lon = -lon;
		s_gpsData.longitude = (float)lon;
		s_gpsData.lonE7     = GPS_DegToE7(lon);
	}
	else GPS_Field(&p);

	s_gpsData.speedKnots = (float)GPS_ParseDouble(GPS_Field(&p));	// 速度(节)
	s_gpsData.speedKmh   = s_gpsData.speedKnots * 1.852f;			// 换算km/h
	s_gpsData.course     = (float)GPS_ParseDouble(GPS_Field(&p));	// 航向角

	date = (uint32_t)GPS_ParseInt(GPS_Field(&p));			// 日期 ddmmyy
	if (date > 0)
	{
		s_gpsData.day   = (uint8_t)(date / 10000);
		s_gpsData.month = (uint8_t)(date / 100 % 100);
		s_gpsData.year  = (uint16_t)(2000 + date % 100);
	}
}

/*---------------- 对外接口 ----------------*/

/*************************************************************************************************
*	函 数 名:	GPS_Init
*
*	函数功能:	初始化USART6(PG9/PG14)及DMA循环接收, 开启空闲中断
*
*************************************************************************************************/
void GPS_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef   DMA_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(GPS_GPIO_CLK | GPS_DMA_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(GPS_USART_CLK, ENABLE);

	/* IO复用 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = GPS_TX_PIN | GPS_RX_PIN;
	GPIO_Init(GPS_TX_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPS_TX_PORT, GPS_TX_PINSRC, GPS_AF);
	GPIO_PinAFConfig(GPS_RX_PORT, GPS_RX_PINSRC, GPS_AF);

	/* 串口参数: 8N1, 无流控 */
	USART_InitStructure.USART_BaudRate   = GPS_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits   = USART_StopBits_1;
	USART_InitStructure.USART_Parity     = USART_Parity_No;
	USART_InitStructure.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(GPS_USART, &USART_InitStructure);

	/* DMA循环接收 */
	DMA_DeInit(GPS_DMA_STREAM);
	DMA_InitStructure.DMA_Channel            = GPS_DMA_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&GPS_USART->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)s_dmaBuf;
	DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize         = GPS_DMA_BUF_SIZE;
	DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
	DMA_Init(GPS_DMA_STREAM, &DMA_InitStructure);
	DMA_Cmd(GPS_DMA_STREAM, ENABLE);

	USART_DMACmd(GPS_USART, USART_DMAReq_Rx, ENABLE);

	/* 空闲中断: 一批语句接收完成后触发 */
	USART_ITConfig(GPS_USART, USART_IT_IDLE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = GPS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(GPS_USART, ENABLE);
}

/*************************************************************************************************
*	函 数 名:	GPS_Process
*
*	返 回 值:	1 - 本次至少解析了一条语句, 0 - 队列为空
*
*	函数功能:	主循环调用, 校验并解析队列里的NMEA语句(GGA/RMC),
*				结果写入GPS_GetData()结构体
*
*	说    明:	每次最多解析 GPS_PARSE_BURST 条, 避免一次突发语句较多时
*				长时间占用主循环; 但足够在两次调用之间把队列排空。
*
*************************************************************************************************/
uint8_t GPS_Process(void)
{
	char *type;
	uint8_t n = 0;

	/* 兜底消费DMA数据(关空闲中断防与ISR竞争, 挂起的中断会在恢复后触发) */
	USART_ITConfig(GPS_USART, USART_IT_IDLE, DISABLE);
	GPS_DmaDrain();
	USART_ITConfig(GPS_USART, USART_IT_IDLE, ENABLE);

	/* 单生产者(中断写入队)/单消费者(主循环出队)环形队列,
	 * 双方只写各自的指针, 字节读写本身原子, 无需临界区 */
	while (s_qRd != s_qWr && n < GPS_PARSE_BURST)
	{
		char *msg = s_lineQ[s_qRd];

		s_qRd = (uint8_t)((s_qRd + 1) % GPS_LINE_QUEUE);
		n++;

		if (!GPS_CheckSumOk(msg)) continue;		// 校验和不对, 整条丢弃

		type = &msg[3];	// 跳过$和两位Talker ID, 兼容GP/GL/BD/GN

		if (type[0] == 'G' && type[1] == 'G' && type[2] == 'A')
		{
			GPS_ParseGGA(msg);
			s_gpsData.updated = 1;
		}
		else if (type[0] == 'R' && type[1] == 'M' && type[2] == 'C')
		{
			GPS_ParseRMC(msg);
			s_gpsData.updated    = 1;
			s_gpsData.fixUpdated = 1;	// RMC才是位置/有效性的权威来源
			s_rmcCount++;
		}
	}

	return (n > 0) ? 1 : 0;
}

/*************************************************************************************************
*	函 数 名:	GPS_GetRmcCount
*
*	返 回 值:	已解析的RMC语句条数
*
*	说    明:	诊断用: 正常定位时该值应该每秒+1; 一直是0说明模块没输出RMC
*				或者校验和/接线有问题, 那么GPS定位就不会生效。
*
*************************************************************************************************/
uint32_t GPS_GetRmcCount(void)
{
	return s_rmcCount;
}

/*************************************************************************************************
*	函 数 名:	GPS_GetQueueDropped
*
*	返 回 值:	因队列满而被丢弃的语句条数(用于诊断丢数据)
*
*************************************************************************************************/
uint16_t GPS_GetQueueDropped(void)
{
	return s_qDropped;
}

/*************************************************************************************************
*	函 数 名:	GPS_GetData
*
*	返 回 值:	定位数据结构体指针
*
*************************************************************************************************/
GPS_DataTypeDef *GPS_GetData(void)
{
	return &s_gpsData;
}

/*************************************************************************************************
*	函 数 名:	GPS_SendString
*
*	函数功能:	阻塞发送字符串, 可用于下发模块配置命令
*
*************************************************************************************************/
void GPS_SendString(char *str)
{
	while (*str)
	{
		USART_SendData(GPS_USART, (uint8_t)*str++);
		while (USART_GetFlagStatus(GPS_USART, USART_FLAG_TXE) == RESET);
	}
}
