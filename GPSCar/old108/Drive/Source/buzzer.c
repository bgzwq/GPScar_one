/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   无源蜂鸣器驱动 (PF10)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. PF10 没有定时器复用功能, 用 TIM11 更新中断翻转 IO 产生方波:
	*     中断频率 = 2 x 音调频率, 占空比固定50%
	*  2. 若使用的是有源蜂鸣器, 调用 Buzzer_On(任意频率) 也会响, 只是音调不可变
	*  3. 音调范围 100~10000Hz, 中断频率最高20kHz, 168MHz下占用可忽略
	*
	************************************************************************************************
***/

#include "buzzer.h"
#include "delay.h"
#include "timer.h"

/* 非阻塞鸣叫状态 */
static uint32_t s_beepEndMs = 0;	// 本次鸣叫结束时刻(0=空闲)

/*************************************************************************************************
*	函 数 名:	Buzzer_Init
*
*	函数功能:	初始化 PF10 推挽输出 + TIM11 时基(更新中断默认关闭=静音)
*
*************************************************************************************************/
void Buzzer_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(BUZZER_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(BUZZER_TIM_CLK, ENABLE);

	/* PF10 推挽输出, 默认低电平(不响) */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	// 最高20kHz翻转, 2MHz足够
	GPIO_InitStructure.GPIO_Pin   = BUZZER_PIN;
	GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);

	/* 168MHz / 168 = 1MHz 计数 */
	TIM_TimeBaseStructure.TIM_Prescaler     = 168 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period        = 1000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(BUZZER_TIM, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);
	TIM_ClearITPendingBit(BUZZER_TIM, TIM_IT_Update);	// 初始化产生的更新事件清掉

	/* 更新中断: 翻转蜂鸣器引脚 */
	NVIC_InitStructure.NVIC_IRQChannel = BUZZER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(BUZZER_TIM, ENABLE);	// 计数器常开, 中断不开=静音
}

/*************************************************************************************************
*	函 数 名:	TIM1_TRG_COM_TIM11_IRQHandler
*
*	函数功能:	TIM11更新中断, 翻转蜂鸣器引脚产生方波
*
*************************************************************************************************/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	if (TIM_GetITStatus(BUZZER_TIM, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(BUZZER_TIM, TIM_IT_Update);
		GPIO_ToggleBits(BUZZER_PORT, BUZZER_PIN);
	}
}

/*************************************************************************************************
*	函 数 名:	Buzzer_On
*
*	入口参数:	freq - 鸣叫频率(Hz), 有效范围约 100~10000
*
*************************************************************************************************/
void Buzzer_On(uint16_t freq)
{
	uint32_t halfPeriod;

	if (freq < 100)   freq = 100;
	if (freq > 10000) freq = 10000;

	halfPeriod = 1000000UL / (2UL * freq);	// 1MHz计数下半周期(翻转间隔)

	TIM_ITConfig(BUZZER_TIM, TIM_IT_Update, DISABLE);	// 先关中断再改ARR, 避免半截波形
	GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);			// 从低电平开始
	TIM_SetAutoreload(BUZZER_TIM, halfPeriod - 1);
	TIM_SetCounter(BUZZER_TIM, 0);
	TIM_ClearITPendingBit(BUZZER_TIM, TIM_IT_Update);
	TIM_ITConfig(BUZZER_TIM, TIM_IT_Update, ENABLE);
}

/*************************************************************************************************
*	函 数 名:	Buzzer_Off
*
*	函数功能:	关闭更新中断, 引脚停低电平, 静音
*
*************************************************************************************************/
void Buzzer_Off(void)
{
	TIM_ITConfig(BUZZER_TIM, TIM_IT_Update, DISABLE);
	GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

/*************************************************************************************************
*	函 数 名:	Buzzer_Beep
*
*	入口参数:	freq - 频率(Hz)
*				ms   - 时长(毫秒), 阻塞
*
*************************************************************************************************/
void Buzzer_Beep(uint16_t freq, uint16_t ms)
{
	Buzzer_On(freq);
	Delay_ms(ms);
	Buzzer_Off();
}

/*************************************************************************************************
*	函 数 名:	Buzzer_BeepAsync
*
*	入口参数:	freq - 频率(Hz)
*				ms   - 时长(毫秒)
*
*	函数功能:	非阻塞鸣叫: 立刻开始发声并登记结束时刻, 不等待
*
*	说    明:	需要在主循环里反复调用 Buzzer_Task() 来关闭声音
*
*************************************************************************************************/
void Buzzer_BeepAsync(uint16_t freq, uint16_t ms)
{
	if (ms == 0) return;

	Buzzer_On(freq);
	s_beepEndMs = Get_Ms() + ms;
	if (s_beepEndMs == 0) s_beepEndMs = 1;	// 0保留为"空闲"标志
}

/*************************************************************************************************
*	函 数 名:	Buzzer_Task
*
*	函数功能:	非阻塞鸣叫的收尾: 到时间自动静音, 主循环里调用
*
*************************************************************************************************/
void Buzzer_Task(void)
{
	if (s_beepEndMs == 0) return;

	if ((int32_t)(Get_Ms() - s_beepEndMs) >= 0)
	{
		Buzzer_Off();
		s_beepEndMs = 0;
	}
}
