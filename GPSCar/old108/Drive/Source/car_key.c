/***
	*************************************************************************************************
	*	@file  	car_key.c
	*	@brief   车身4路按键 (PE2/PE3/PG5/PG7, 原KEY5=PG8与电机M3冲突已移除)
   *************************************************************************************************
   *  @description
	*
	*  1. 上拉输入, 按下接地为低电平
	*  2. 【非阻塞】扫描: CarKey_Poll() 每50ms调一次, 内部做消抖与长按判定,
	*     事件写入小队列, 由 CarKey_GetEvent() 取走。
	*     旧代码里 "while(按下);" 等松手的写法会阻塞主循环(GPS/K230/遥控全部停摆),
	*     如果按键卡住还会永久死循环, 这里已经去掉。
	*  3. 短按 -> CAR_KEY_EVENT_PRESS, 按住 >=1s -> CAR_KEY_EVENT_LONG
	*
	************************************************************************************************
***/

#include "car_key.h"
#include "timer.h"

/*---------------------- 参数 ----------------------------*/
#define CAR_KEY_DEBOUNCE_MS		20		// 消抖时间
#define CAR_KEY_LONG_MS			1000	// 长按判定时间
#define CAR_KEY_EVENT_QUEUE		8		// 事件队列深度

/*---------------------- 静态状态 ----------------------------*/
typedef struct
{
	GPIO_TypeDef *port;
	uint16_t      pin;
	uint32_t      clk;
	uint8_t       id;

	uint8_t       raw;			// 上一次原始采样电平(0=按下 1=松开)
	uint32_t      rawMs;		// 原始电平发生变化的时间戳
	uint8_t       stable;		// 消抖后的电平(0=按下 1=松开)
	uint32_t      pressMs;		// 本次按下的时间戳
	uint8_t       longSent;		// 本次长按是否已上报
} CarKeyHW_TypeDef;

static CarKeyHW_TypeDef s_keyHW[CAR_KEY_NUM] =
{
	{ GPIOE, GPIO_Pin_2, RCC_AHB1Periph_GPIOE, CAR_KEY1, 1, 0, 1, 0, 0 },
	{ GPIOE, GPIO_Pin_3, RCC_AHB1Periph_GPIOE, CAR_KEY2, 1, 0, 1, 0, 0 },
	{ GPIOG, GPIO_Pin_5, RCC_AHB1Periph_GPIOG, CAR_KEY3, 1, 0, 1, 0, 0 },
	{ GPIOG, GPIO_Pin_7, RCC_AHB1Periph_GPIOG, CAR_KEY4, 1, 0, 1, 0, 0 },
};

/* 事件队列: 每项高4位=事件类型, 低4位=键号 */
static uint8_t s_evQ[CAR_KEY_EVENT_QUEUE];
static uint8_t s_evWr = 0;
static uint8_t s_evRd = 0;

/*---------------------- 内部函数 ----------------------------*/

static void CarKey_PushEvent(uint8_t id, uint8_t type)
{
	uint8_t next = (uint8_t)((s_evWr + 1) % CAR_KEY_EVENT_QUEUE);

	if (next == s_evRd) return;		// 队列满, 丢弃(正常不会发生)

	s_evQ[s_evWr] = (uint8_t)((type << 4) | (id & 0x0F));
	s_evWr = next;
}

/*---------------------- 对外接口 ----------------------------*/

/*************************************************************************************************
*	函 数 名:	CarKey_Init
*
*	函数功能:	初始化4个按键引脚为上拉输入
*
*************************************************************************************************/
void CarKey_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint8_t i;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOG, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	for (i = 0; i < CAR_KEY_NUM; i++)
	{
		s_keyHW[i].raw      = 1;
		s_keyHW[i].stable   = 1;
		s_keyHW[i].rawMs    = 0;
		s_keyHW[i].pressMs  = 0;
		s_keyHW[i].longSent = 0;
	}
	s_evWr = 0;
	s_evRd = 0;
}

/*************************************************************************************************
*	函 数 名:	CarKey_Poll
*
*	函数功能:	非阻塞按键扫描: 消抖 + 长按判定, 每50ms调用一次
*
*************************************************************************************************/
void CarKey_Poll(void)
{
	uint32_t now = Get_Ms();
	uint8_t  i;

	for (i = 0; i < CAR_KEY_NUM; i++)
	{
		/* 按下为低电平 -> level 0 表示按下 */
		uint8_t level = (GPIO_ReadInputDataBit(s_keyHW[i].port, s_keyHW[i].pin) == 0) ? 0 : 1;

		if (level != s_keyHW[i].raw)
		{
			/* 原始电平刚变化, 重新计时消抖 */
			s_keyHW[i].raw   = level;
			s_keyHW[i].rawMs = now;
			continue;
		}

		if ((uint32_t)(now - s_keyHW[i].rawMs) < CAR_KEY_DEBOUNCE_MS) continue;	// 还没稳定
		if (level == s_keyHW[i].stable) continue;								// 无变化

		s_keyHW[i].stable = level;

		if (level == 0)				// 消抖后的"按下"
		{
			s_keyHW[i].pressMs  = now;
			s_keyHW[i].longSent = 0;
			CarKey_PushEvent(s_keyHW[i].id, CAR_KEY_EVENT_PRESS);
		}
		/* 松手不需要事件 */
	}

	/* 长按判定 */
	for (i = 0; i < CAR_KEY_NUM; i++)
	{
		if (s_keyHW[i].stable == 0 && !s_keyHW[i].longSent &&
			(uint32_t)(now - s_keyHW[i].pressMs) >= CAR_KEY_LONG_MS)
		{
			s_keyHW[i].longSent = 1;
			CarKey_PushEvent(s_keyHW[i].id, CAR_KEY_EVENT_LONG);
		}
	}
}

/*************************************************************************************************
*	函 数 名:	CarKey_GetEvent
*
*	入口参数:	key - 输出, 键号 CAR_KEY1~CAR_KEY4
*
*	返 回 值:	CAR_KEY_EVENT_PRESS / CAR_KEY_EVENT_LONG / CAR_KEY_EVENT_NONE
*
*	函数功能:	取出一个按键事件(非阻塞, 队列空立即返回)
*
*************************************************************************************************/
uint8_t CarKey_GetEvent(uint8_t *key)
{
	uint8_t e;

	if (s_evRd == s_evWr) return CAR_KEY_EVENT_NONE;

	e = s_evQ[s_evRd];
	s_evRd = (uint8_t)((s_evRd + 1) % CAR_KEY_EVENT_QUEUE);

	if (key) *key = (uint8_t)(e & 0x0F);
	return (uint8_t)((e >> 4) & 0x0F);
}

/*************************************************************************************************
*	函 数 名:	CarKey_Scan
*
*	返 回 值:	有新的"短按"返回键号 1~4, 否则返回 0
*
*	函数功能:	兼容旧接口的非阻塞封装(长按事件不会被消费, 留给 CarKey_GetEvent)
*
*************************************************************************************************/
uint8_t CarKey_Scan(void)
{
	uint8_t e;

	if (s_evRd == s_evWr) return 0;

	e = s_evQ[s_evRd];
	if (((e >> 4) & 0x0F) != CAR_KEY_EVENT_PRESS) return 0;

	s_evRd = (uint8_t)((s_evRd + 1) % CAR_KEY_EVENT_QUEUE);
	return (uint8_t)(e & 0x0F);
}

/*************************************************************************************************
*	函 数 名:	CarKey_GetState
*
*	入口参数:	id - 键号 CAR_KEY1 ~ CAR_KEY4
*
*	返 回 值:	1 - 正在按下, 0 - 松开
*
*	函数功能:	读取按键实时电平(无消抖), 供菜单显示当前状态
*
*************************************************************************************************/
uint8_t CarKey_GetState(uint8_t id)
{
	uint8_t i;

	for (i = 0; i < CAR_KEY_NUM; i++)
	{
		if (s_keyHW[i].id == id)
		{
			return (GPIO_ReadInputDataBit(s_keyHW[i].port, s_keyHW[i].pin) == 0) ? 1 : 0;
		}
	}
	return 0;
}
