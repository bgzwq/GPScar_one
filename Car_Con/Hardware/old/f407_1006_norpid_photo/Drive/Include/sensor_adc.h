#ifndef __SENSOR_ADC_H
#define __SENSOR_ADC_H

#include "stm32f4xx.h"

/*---------------------- ADC采样通道 ----------------------------*/
/*
 * ADC1 + DMA2 Stream0 循环扫描, 4路模拟输入:
 *   PA3 = ADC123_IN3  电源检测1
 *   PA4 = ADC12_IN4   电源检测2
 *   PB0 = ADC12_IN8   亮度检测
 *   PC4 = ADC12_IN14  湿度检测
 */
typedef enum
{
	ADC_CH_POWER1 = 0,	// 电源检测1
	ADC_CH_POWER2,		// 电源检测2
	ADC_CH_LIGHT,		// 亮度
	ADC_CH_HUMIDITY,	// 湿度
	ADC_CH_NUM
} SensorADC_TypeDef;

#define ADC_REF_VOLTAGE		3.3f	// 参考电压
#define ADC_FULL_SCALE		4095.0f	// 12位满量程

/*---------------------- 函数声明 ----------------------------*/

void 	SensorADC_Init(void);							// ADC+DMA初始化
uint16_t SensorADC_GetRaw(SensorADC_TypeDef ch);		// 读取原始值 0~4095
float 	SensorADC_GetVoltage(SensorADC_TypeDef ch);		// 读取引脚电压(V)

#endif //__SENSOR_ADC_H
