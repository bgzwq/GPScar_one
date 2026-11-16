#ifndef __ESTOP_H
#define __ESTOP_H

#include "stm32f4xx.h"

#define ESTOP_SRC_GPIO   0x01
#define ESTOP_SRC_SOFT   0x02
#define ESTOP_SRC_ULTRA  0x04
#define ESTOP_SRC_LINK   0x08

void Estop_Init(void);
void Estop_Task(uint32_t now);
void Estop_Request(uint8_t source);
uint8_t Estop_Reset(uint8_t source);
uint8_t Estop_IsActive(void);
uint8_t Estop_GetSource(void);
uint32_t Estop_GetCount(void);

#endif
