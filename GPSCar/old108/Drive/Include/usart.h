#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "stm32f4xx.h"

/*----------------------USART���ú� ------------------------*/

#define  USART1_BaudRate  9600

#define  USART1_TX_PIN				GPIO_Pin_9					// TX ����
#define	USART1_TX_PORT				GPIOA							// TX ���Ŷ˿�
#define	USART1_TX_CLK				RCC_AHB1Periph_GPIOA		// TX ����ʱ��
#define  USART1_TX_PinSource     GPIO_PinSource9			// ����Դ

#define  USART1_RX_PIN				GPIO_Pin_10             // RX ����
#define	USART1_RX_PORT				GPIOA                   // RX ���Ŷ˿�
#define	USART1_RX_CLK				RCC_AHB1Periph_GPIOA    // RX ����ʱ��
#define  USART1_RX_PinSource     GPIO_PinSource10        // ����Դ


/*---------------------- �������� ----------------------------*/

void  Usart_Config (void);	// USART��ʼ������

#endif //__USART_H

