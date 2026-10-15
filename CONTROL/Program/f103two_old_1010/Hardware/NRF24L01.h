#ifndef __NRF24L01_H
#define __NRF24L01_H

#include <stdint.h>

/*用户配置（接收端必须与此一致）*/
#define NRF24L01_CHANNEL	76								//射频频道：2400+76 = 2476MHz
#define NRF24L01_ADDR		{0x11, 0x22, 0x33, 0x44, 0x55}	//5字节通信地址
#define NRF24L01_PAYLOAD	8								//有效载荷长度（字节）

uint8_t NRF24L01_Init(void);
uint8_t NRF24L01_Check(void);
uint8_t NRF24L01_SendPacket(uint8_t *Data, uint8_t Length);

#endif
