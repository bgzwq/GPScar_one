#ifndef __REMOTE_UI_H
#define __REMOTE_UI_H

#include <stdint.h>

/*NRF24链路状态*/
#define UI_NRF_OFFLINE	0	//模块未检测到
#define UI_NRF_LOST		1	//模块在线，但ACK丢失（接收端不在线）
#define UI_NRF_OK		2	//模块在线，ACK正常

void UI_Init(void);
void UI_Update(uint8_t *Joy, uint8_t KeyState, uint8_t NrfStatus, uint8_t BtState, uint32_t TxCount, uint8_t TxKey);

#endif
