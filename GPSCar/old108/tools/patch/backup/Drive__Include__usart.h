#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f4xx.h"

/*----------------------USART锟斤拷锟矫猴拷 ------------------------*/

/* USART1 = 调试 printf + K230 视觉, 两者共用一根线。
 * 【2026-11 统一】K230 协议(K230.h)写的是 115200, 原来这里是 9600,
 * 结果 K230 帧全是乱码, 而且 printf 一个字符要 1.04ms(整屏菜单要几百毫秒)。
 * 这里改成 115200: printf 快 12 倍, 与 K230 协议一致。
 * 如果 K230 端确实是 9600, 只改这一行即可(但 printf 会变慢)。 */
#define  USART1_BaudRate  115200

#define  USART1_TX_PIN				GPIO_Pin_9					// TX 锟斤拷锟斤拷
#define	USART1_TX_PORT				GPIOA							// TX 锟斤拷锟脚端匡拷
#define	USART1_TX_CLK				RCC_AHB1Periph_GPIOA		// TX 锟斤拷锟斤拷时锟斤拷
#define  USART1_TX_PinSource     GPIO_PinSource9			// 锟斤拷锟斤拷源

#define  USART1_RX_PIN				GPIO_Pin_10             // RX 锟斤拷锟斤拷
#define	USART1_RX_PORT				GPIOA                   // RX 锟斤拷锟脚端匡拷
#define	USART1_RX_CLK				RCC_AHB1Periph_GPIOA    // RX 锟斤拷锟斤拷时锟斤拷
#define  USART1_RX_PinSource     GPIO_PinSource10        // 锟斤拷锟斤拷源


/*---------------------- 锟斤拷锟斤拷锟斤拷锟斤拷 ----------------------------*/

void  Usart_Config (void);	// USART锟斤拷始锟斤拷锟斤拷锟斤拷

#endif //__USART_H

