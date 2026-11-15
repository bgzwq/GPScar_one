/***
	***************************************************************************
	*	@file  	delay.c
	*	@brief   delay接口相关函数
   ***************************************************************************
   *  @description
	*
	*  【RTOS 兼容改动】
	*  原来这里把 SysTick 配成 1ms 中断、用 TimingDelay 计数来实现 Delay_ms()。
	*  但 SysTick 现在归 FreeRTOS 内核所有(configTICK_RATE_HZ=1000, 1 tick = 1ms),
	*  两者不可能共存。因此:
	*    1) Delay_Init() 不再配置 SysTick(配了也会被 vTaskStartScheduler 覆盖),
	*       它只负责把 TIM2 微秒时基准备好;
	*    2) Delay_ms() 改成"双路径":
	*         - 调度器已运行 且 不在中断里 -> vTaskDelay(), 把 CPU 让给别的任务
	*         - 启动阶段(调度器未运行)或中断里 -> 用 TIM2 时基忙等
	*       这样上层 12 处 Delay_ms() 调用一行都不用改;
	*    3) TimingDelay_Decrement() 已删除(它只服务于旧的 SysTick 计数)。
	*  Delay_us() 定义在 timer.c 里, 基于 TIM2, 完全不受影响。
	*
	***************************************************************************
***/

#include "delay.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "task.h"

//	函数：延时初始化
//	说明：SysTick 已交给 FreeRTOS，这里只保证 TIM2 微秒时基就绪
//
void Delay_Init(void)
{
	/* Timer_Init() 是幂等的: bsp.c 里紧接着还会再调一次, 重复调用无害 */
	Timer_Init();
}

//	函数：毫秒延时(RTOS 兼容)
// 参数：nTime - 延时时间，单位ms
//	说明：调度器运行中则转成 vTaskDelay 让出 CPU；否则用 TIM2 忙等。
//	      注意 nTime 需 < 4294967 (乘 1000 不能溢出)，实际用法远小于此。
//
void Delay_ms(uint32_t nTime)
{
	uint32_t start;

	if (nTime == 0) return;

	/* 调度器已启动、且当前不在中断上下文 -> 用 RTOS 延时 */
	if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && __get_IPSR() == 0)
	{
		vTaskDelay(pdMS_TO_TICKS(nTime));
		return;
	}

	/* 启动阶段或中断里 -> 用 TIM2 微秒时基忙等 */
	start = Get_Us();
	while ((uint32_t)(Get_Us() - start) < (nTime * 1000u))
	{
	}
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
