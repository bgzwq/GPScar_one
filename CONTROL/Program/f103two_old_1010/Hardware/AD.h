#ifndef __AD_H
#define __AD_H

#include <stdint.h>

/*摇杆ADC通道索引（本遥控器只使用1个摇杆）*/
#define AD_J1X		0		//摇杆1 X轴，PA0，ADC1_IN0
#define AD_J1Y		1		//摇杆1 Y轴，PA1，ADC1_IN1

void AD_Init(void);
uint16_t AD_GetValue(uint8_t Channel);
uint8_t AD_GetMapped(uint8_t Channel);

#endif
