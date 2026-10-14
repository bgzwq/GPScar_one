/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   无源蜂鸣器驱动 (TIM11_CH1 = PF6)
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. PWM频率即音调, 占空比固定50%
	*  2. 若使用的是有源蜂鸣器, 调用 Buzzer_On(任意频率) 也会响, 只是音调不可变
	*
	*  【蜂鸣器已整体禁用 —— 供电功率不足】
	*  Buzzer_Init/Buzzer_On/Buzzer_Off 函数体已注释: 引脚不初始化、定时器不开,
	*  上层所有调用(main.c提示音、过近警报等)变成空操作, 不再耗电。
	*  恢复方法: 取消下面三个函数体内的注释即可, 调用处无需改动。
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
*	函数功能:	初始化 TIM11_CH1, 默认输出关闭(静音)
*
*************************************************************************************************/
void Buzzer_Init(void)
{
	/* 已禁用(省电): 不初始化PF6与TIM11, 引脚保持复位态零驱动
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(BUZZER_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(BUZZER_TIM_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Pin   = BUZZER_PIN;
	GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(BUZZER_PORT, BUZZER_PinSource, BUZZER_AF);

	TIM_TimeBaseStructure.TIM_Prescaler     = 168 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period        = 1000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(BUZZER_TIM, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse       = 500;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OC1Init(BUZZER_TIM, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(BUZZER_TIM, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);
	TIM_Cmd(BUZZER_TIM, ENABLE);
	*/
}

/*************************************************************************************************
*	函 数 名:	Buzzer_On
*
*	入口参数:	freq - 鸣叫频率(Hz), 有效范围约 100~10000
*
*************************************************************************************************/
void Buzzer_On(uint16_t freq)
{
	(void)freq;
	/* 已禁用(省电): 空操作
	uint32_t period;

	if (freq < 100)   freq = 100;
	if (freq > 10000) freq = 10000;

	period = 1000000UL / freq;

	TIM_SetAutoreload(BUZZER_TIM, period - 1);
	TIM_SetCompare1(BUZZER_TIM, period / 2);
	TIM_SetCounter(BUZZER_TIM, 0);
	TIM_CCxCmd(BUZZER_TIM, TIM_Channel_1, TIM_CCx_Enable);
	*/
}

/*************************************************************************************************
*	函 数 名:	Buzzer_Off
*
*	函数功能:	关闭PWM输出, 静音
*
*************************************************************************************************/
void Buzzer_Off(void)
{
	/* 已禁用(省电): 空操作
	TIM_CCxCmd(BUZZER_TIM, TIM_Channel_1, TIM_CCx_Disable);
	*/
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
