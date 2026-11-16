#ifndef __UART_HOST_H
#define __UART_HOST_H

#include "stm32f4xx.h"

#define UART_HOST_LINE_SIZE 192

typedef struct
{
    uint32_t rxLines;
    uint32_t rxCommands;
    uint32_t rxErrors;
    uint32_t txTelemetry;
    uint32_t lastRxMs;
    uint8_t wifi;
    uint8_t bluetooth;
    uint8_t voice;
    char lastCommand[32];
    char lastRx[64];
    char lastTx[80];
} UARTHost_Status_TypeDef;

void UARTHost_Init(void);
void UARTHost_Process(uint32_t now);
void UARTHost_SendTelemetry(uint32_t now);
void UARTHost_SendEvent(const char *event, const char *detail);
UARTHost_Status_TypeDef *UARTHost_GetStatus(void);
uint8_t Command_Exec(const char *line, const char *source);

#endif
