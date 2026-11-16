#ifndef __LIDAR_H
#define __LIDAR_H

#include "stm32f4xx.h"

#define LIDAR_SECTORS 360
#define LIDAR_DMA_SIZE 512

typedef struct
{
    uint16_t distanceMm[LIDAR_SECTORS];
    uint32_t packets;
    uint32_t crcErrors;
    uint32_t lastPacketMs;
    uint16_t speed;
    uint8_t online;
} Lidar_DataTypeDef;

void Lidar_Init(void);
void Lidar_Process(uint32_t now);
Lidar_DataTypeDef *Lidar_GetData(void);
uint16_t Lidar_GetMinDistance(uint16_t center, uint16_t halfWidth);

#endif
