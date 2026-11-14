#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f4xx.h"

/*---------------------- ����/WiFi�������ú� ------------------------*/
/*
 * USART2: TX=PD5, RX=PD6, ������WiFiģ�鹲�ô˴���
 * Ĭ�ϲ����� 115200, �����·����޸���ƥ��ģ��
 */
#define BT_BaudRate			9600

#define BT_USART			USART2
#define BT_USART_CLK		RCC_APB1Periph_USART2
#define BT_IRQn				USART2_IRQn

#define BT_TX_PIN			GPIO_Pin_5
#define BT_TX_PORT			GPIOD
#define BT_TX_PinSource		GPIO_PinSource5
#define BT_RX_PIN			GPIO_Pin_6
#define BT_RX_PORT			GPIOD
#define BT_RX_PinSource		GPIO_PinSource6
#define BT_GPIO_CLK			RCC_AHB1Periph_GPIOD
#define BT_AF				GPIO_AF_USART2

#define BT_LINE_BUF_SIZE	64		// ����ָ��峤��

/*---------------------- �������� ----------------------------*/

void 	BT_Init(void);							// ����/WiFi���ڳ�ʼ��
void 	BT_SendByte(uint8_t byte);				// ���͵��ֽ�
void 	BT_SendString(char *str);				// �����ַ���
void 	BT_SendData(uint8_t *data, uint16_t len);	// �������ݿ�
uint8_t BT_ReadLine(char *buf, uint8_t maxLen);	// ��ȡһ��ָ��, ����1������
uint8_t BT_ReadByte(uint8_t *byte);				// ��ȡ���ֽ�, ����1������

#endif //__BLUETOOTH_H
