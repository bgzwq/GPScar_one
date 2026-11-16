#ifndef __TIMER_H
#define __TIMER_H
#include "stm32f4xx.h"
typedef struct { int GPIO_Mode, GPIO_OType, GPIO_PuPd, GPIO_Speed, GPIO_Pin; } GPIO_InitTypeDef;
typedef struct { int USART_BaudRate, USART_WordLength, USART_StopBits, USART_Parity, USART_Mode, USART_HardwareFlowControl; } USART_InitTypeDef;
typedef struct { int DMA_Channel, DMA_DIR, DMA_BufferSize, DMA_PeripheralInc, DMA_MemoryInc, DMA_PeripheralDataSize, DMA_MemoryDataSize, DMA_Mode, DMA_Priority, DMA_FIFOMode, DMA_FIFOThreshold, DMA_MemoryBurst, DMA_PeripheralBurst; uint32_t DMA_PeripheralBaseAddr, DMA_Memory0BaseAddr; } DMA_InitTypeDef;
typedef struct { int NVIC_IRQChannel, NVIC_IRQChannelPreemptionPriority, NVIC_IRQChannelSubPriority, NVIC_IRQChannelCmd; } NVIC_InitTypeDef;
typedef struct { uint32_t DR; } USART_TypeDef;
static USART_TypeDef usart3;
#define USART3 (&usart3)
#define RCC_AHB1Periph_GPIOD 1
#define RCC_AHB1Periph_DMA1 2
#define RCC_APB1Periph_USART3 4
#define GPIO_Mode_AF 0
#define GPIO_OType_PP 0
#define GPIO_PuPd_UP 0
#define GPIO_Speed_50MHz 0
#define GPIO_Pin_8 8
#define GPIO_Pin_9 9
#define GPIOD 0
#define GPIO_PinSource8 8
#define GPIO_PinSource9 9
#define GPIO_AF_USART3 7
#define USART_WordLength_8b 0
#define USART_StopBits_1 0
#define USART_Parity_No 0
#define USART_Mode_Rx 1
#define USART_Mode_Tx 2
#define USART_HardwareFlowControl_None 0
#define DMA1_Stream1 0
#define DMA_Channel_4 4
#define DMA_DIR_PeripheralToMemory 0
#define DMA_PeripheralInc_Disable 0
#define DMA_MemoryInc_Enable 1
#define DMA_PeripheralDataSize_Byte 0
#define DMA_MemoryDataSize_Byte 0
#define DMA_Mode_Circular 1
#define DMA_Priority_High 1
#define DMA_FIFOMode_Disable 0
#define DMA_FIFOThreshold_Full 0
#define DMA_MemoryBurst_Single 0
#define DMA_PeripheralBurst_Single 0
#define USART_DMAReq_Rx 1
#define USART_IT_IDLE 1
#define USART3_IRQn 1
static inline void RCC_AHB1PeriphClockCmd(int a, int b) {(void)a;(void)b;}
static inline void RCC_APB1PeriphClockCmd(int a, int b) {(void)a;(void)b;}
static inline void GPIO_Init(int a, GPIO_InitTypeDef *b) {(void)a;(void)b;}
static inline void GPIO_PinAFConfig(int a, int b, int c) {(void)a;(void)b;(void)c;}
static inline void USART_Init(USART_TypeDef *a, USART_InitTypeDef *b) {(void)a;(void)b;}
static inline void DMA_DeInit(int a) {(void)a;}
static inline int DMA_GetCmdStatus(int a) {(void)a;return DISABLE;}
static inline void DMA_Init(int a, DMA_InitTypeDef *b) {(void)a;(void)b;}
static inline void DMA_Cmd(int a, int b) {(void)a;(void)b;}
static inline void USART_DMACmd(USART_TypeDef *a, int b, int c) {(void)a;(void)b;(void)c;}
static inline void USART_ITConfig(USART_TypeDef *a, int b, int c) {(void)a;(void)b;(void)c;}
static inline void NVIC_Init(NVIC_InitTypeDef *a) {(void)a;}
static inline void USART_Cmd(USART_TypeDef *a, int b) {(void)a;(void)b;}
static inline uint16_t DMA_GetCurrDataCounter(int a) {(void)a;return LIDAR_DMA_SIZE;}
#endif
