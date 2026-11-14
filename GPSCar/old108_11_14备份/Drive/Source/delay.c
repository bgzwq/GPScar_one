/***
	***************************************************************************
	*	@file  	delay.c
	*	@brief   delay接口相关函数
   ***************************************************************************
   *  @description
	*
	*  SysTick定时器配置为1ms中断，实现毫秒延时
	* 	
	***************************************************************************
***/

#include "delay.h"
#include "timer.h"

static __IO uint32_t TimingDelay;  //计数变量


//	函数：延时初始化
//	说明：配置 SysTick 为1ms中断，并启动定时器
//
void Delay_Init(void)
{
	SysTick_Config(SystemCoreClock / 1000);  //配置SysTick时钟为1ms中断
}

//	函数：计时函数
//	说明：在 SysTick 中断服务函数里被调用
//
void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0)
	{ 
		TimingDelay--;
	}
}

//	函数：毫秒延时
// 参数：nTime - 延时时间，单位ms
//	说明：每次调用都会重新给TimingDelay赋值，实现 n 毫秒的延时，最大延时 4294967295 ms。	
//
void Delay_ms(uint32_t nTime)
{ 
	TimingDelay = nTime;

	while(TimingDelay != 0);
}





/*---------------- 非阻塞延时 (基于TIM2时基, 需先调用Timer_Init) ----------------*/

/*************************************************************************************************
*	函 数 名:	NB_DelayReset
*
*	入口参数:	nb - 非阻塞延时句柄
*
*	函数功能:	停止计时并复位, 下次调用NB_DelayMs/NB_DelayUs时重新开始计时
*
*************************************************************************************************/
void NB_DelayReset(NB_DelayTypeDef *nb)
{
	nb->start   = 0;
	nb->running = 0;
}

/*************************************************************************************************
*	函 数 名:	NB_DelayMs
*
*	入口参数:	nb   - 非阻塞延时句柄(每个任务一个静态变量)
*				nMs  - 间隔时间, 单位ms
*
*	返 回 值:	1 - 时间到(自动重新计时, 可再次进入等待), 0 - 未到时间
*
*	函数功能:	非阻塞毫秒延时, 首次调用自动开始计时;
*				到期后按周期自动重装, 多次调用不会产生累计误差
*
*	用法示例:	static NB_DelayTypeDef ledTimer;
*				if (NB_DelayMs(&ledTimer, 500)) { LED_Toggle(); }	// 每500ms执行一次
*
*************************************************************************************************/
uint8_t NB_DelayMs(NB_DelayTypeDef *nb, uint32_t nMs)
{
	if (!nb->running)
	{
		nb->start   = Get_Ms();
		nb->running = 1;
		return 0;
	}

	if ((uint32_t)(Get_Ms() - nb->start) >= nMs)	// 无符号减法, 可容忍计数器回绕
	{
		nb->start += nMs;							// 按周期重装, 避免累计漂移
		return 1;
	}

	return 0;
}

/*************************************************************************************************
*	函 数 名:	NB_DelayUs
*
*	入口参数:	nb   - 非阻塞延时句柄(每个任务一个静态变量)
*				nUs  - 间隔时间, 单位us
*
*	返 回 值:	1 - 时间到(自动重新计时), 0 - 未到时间
*
*	函数功能:	非阻塞微秒延时, 用法同NB_DelayMs
*
*************************************************************************************************/
uint8_t NB_DelayUs(NB_DelayTypeDef *nb, uint32_t nUs)
{
	if (!nb->running)
	{
		nb->start   = Get_Us();
		nb->running = 1;
		return 0;
	}

	if ((uint32_t)(Get_Us() - nb->start) >= nUs)
	{
		nb->start += nUs;
		return 1;
	}

	return 0;
}
