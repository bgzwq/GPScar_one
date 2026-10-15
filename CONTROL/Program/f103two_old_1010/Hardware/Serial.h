#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdint.h>

#define SERIAL_RX_BUF_SIZE	64		//接收环形缓冲区大小

void Serial_Init(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendPacket(uint8_t *Data, uint8_t Length);
void Serial_SendRemoteFrame(uint8_t X, uint8_t Y, uint8_t Key);
uint8_t Serial_BTConnected(void);
uint8_t Serial_RxAvailable(void);
uint8_t Serial_RxRead(void);

#endif
