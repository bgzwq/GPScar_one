#ifndef __CAR_KEY_H
#define __CAR_KEY_H

#include "stm32f4xx.h"

/*---------------------- 车身按键配置宏 ------------------------*/
/*
 * 4个按键, 另一端接地, 按下为低电平, 上拉输入:
 *   KEY1=PE2  KEY2=PE3  KEY3=PG5  KEY4=PG7
 * (原KEY5=PG8 与电机M3方向脚冲突, 已移除, PG8归电机使用)
 */
#define CAR_KEY1	1
#define CAR_KEY2	2
#define CAR_KEY3	3
#define CAR_KEY4	4
#define CAR_KEY_NUM	4

/*---------------------- 按键事件 ----------------------------*/
/*
 * 【为什么要改成事件方式】
 * 原来的 CarKey_Scan() 里是:
 *     Delay_ms(10);  while (GPIO_ReadInputDataBit(...) == 0);   // 等松手
 * 这是一个死等的阻塞循环: 手按住按键不放时, 主循环里的 GPS解析/K230组帧/
 * 蓝牙遥控/避障状态机全都被卡住; 更糟的是如果按键被卡住或引脚短路,
 * 固件会永久停在 while 里出不来。
 * 现在改成: 每50ms调用 CarKey_Poll() 做一次非阻塞扫描(消抖+长按),
 * 产生的事件放进小队列, 由 CarKey_GetEvent() 取走。
 */
#define CAR_KEY_EVENT_NONE	0	// 无事件
#define CAR_KEY_EVENT_PRESS	1	// 按下(短按)
#define CAR_KEY_EVENT_LONG	2	// 长按(按住 >= CAR_KEY_LONG_MS)

/*---------------------- 函数声明 ----------------------------*/

void 	CarKey_Init(void);					// 按键初始化
void 	CarKey_Poll(void);					// 非阻塞扫描, 需每50ms调用一次
uint8_t CarKey_GetEvent(uint8_t *key);		// 取出一个事件(返回 CAR_KEY_EVENT_*), 0=无
uint8_t CarKey_Scan(void);					// 兼容旧接口: 有新的"短按"返回键号, 不阻塞
uint8_t CarKey_GetState(uint8_t id);		// 读取按键实时状态: 1=按下 0=松开

#endif //__CAR_KEY_H
