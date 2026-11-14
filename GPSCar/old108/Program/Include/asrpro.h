#ifndef __ASRPRO_H
#define __ASRPRO_H

#include "stm32f4xx.h"

/*---------------------- ASRPRO语音模块配置宏 ------------------------*/
/*
 * USART3: TX=PD8, RX=PD9, 接ASRPRO语音识别模块
 * ASRPRO默认波特率常见为 9600, 若模块设置不同请修改下方宏
 */
#define ASR_BaudRate		9600

#define ASR_USART			USART3
#define ASR_USART_CLK		RCC_APB1Periph_USART3
#define ASR_IRQn			USART3_IRQn

#define ASR_TX_PIN			GPIO_Pin_8
#define ASR_TX_PORT			GPIOD
#define ASR_TX_PinSource	GPIO_PinSource8
#define ASR_RX_PIN			GPIO_Pin_9
#define ASR_RX_PORT			GPIOD
#define ASR_RX_PinSource	GPIO_PinSource9
#define ASR_GPIO_CLK		RCC_AHB1Periph_GPIOD
#define ASR_AF				GPIO_AF_USART3

#define ASR_LINE_BUF_SIZE	64		// 单行指令缓冲长度

/*---------------------- 函数声明 ----------------------------*/

void 	ASR_Init(void);							// ASRPRO串口初始化
void 	ASR_SendByte(uint8_t byte);				// 发送单字节
void 	ASR_SendString(char *str);				// 发送字符串
uint8_t ASR_ReadLine(char *buf, uint8_t maxLen);	// 读取一行识别结果, 返回1有新行
uint8_t ASR_ReadByte(uint8_t *byte);			// 读取单字节, 返回1有数据

#endif //__ASRPRO_H
