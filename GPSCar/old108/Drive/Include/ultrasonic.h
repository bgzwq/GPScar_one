#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "stm32f4xx.h"

/*---------------------- 超声波配置宏 ------------------------*/
/*
 * 两个 HC-SR04 超声波模块:
 *   雷达位(舵机上): TRIG=PB15, ECHO=PD14
 *   固定位(车头):   TRIG=PD0,  ECHO=PD1
 * 测距时序用 timer.c 的 1MHz 时基
 */
typedef enum
{
	ULTRA_RADAR = 0,	// 舵机扫描雷达
	ULTRA_FIXED,		// 车头固定
	ULTRA_NUM
} Ultrasonic_TypeDef;

#define ULTRA_TIMEOUT_US	30000	// 回波超时30ms, 对应约5米
#define ULTRA_MAX_CM		400.0f	// 有效量程

/*---------------------- 函数声明 ----------------------------*/

void 	Ultrasonic_Init(void);								// 超声波初始化
float 	Ultrasonic_GetDistance(Ultrasonic_TypeDef id);		// 测距, 返回cm, 超量程返回 ULTRA_MAX_CM

#endif //__ULTRASONIC_H
