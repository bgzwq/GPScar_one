#include <stdio.h>
#include <string.h>
#include "uart_host.h"
#include "menu.h"
#include "car.h"
#include "estop.h"
#include "location.h"
#include "lanyacon.h"
#include "ff.h"

static int failed;
static uint8_t rx[1024], tx[4096];
static uint32_t rxRead, txLen;
static int lastDirection, moveCalls;
static uint8_t stopped;
WorkMode_TypeDef g_mode = MODE_IDLE;
int16_t g_speed = 30;
uint8_t g_basket;
#define CHECK(c) do { if (c) printf("PASS %s\n", #c); else { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); failed++; } } while (0)

uint8_t BT_ReadByte(uint8_t *b) { if (rxRead >= sizeof(rx) || !rx[rxRead]) return 0; *b = rx[rxRead++]; return 1; }
void BT_SendString(char *s) { size_t n = strlen(s); if (txLen + n < sizeof(tx)) { memcpy(tx + txLen, s, n); txLen += n; tx[txLen] = 0; } }
uint8_t LanyaCon_Feed(uint8_t b) { (void)b; return 0; }
void LanyaCon_Task(uint32_t now) { (void)now; }
void Car_Move(CarDir_TypeDef d, int16_t s) { (void)s; lastDirection = d; moveCalls++; stopped = 0; }
void Car_Turn(CarDir_TypeDef d, int16_t s) { Car_Move(d, s); }
void Car_Stop(void) { stopped = 1; }
void Estop_Request(uint8_t s) { (void)s; stopped = 1; }
uint8_t Estop_Reset(uint8_t s) { (void)s; return 1; }
uint8_t Estop_IsActive(void) { return stopped; }
uint8_t Estop_GetSource(void) { return stopped ? 2 : 0; }
void App_SetMode(WorkMode_TypeDef m) { g_mode = m; stopped = 0; }
void App_SwitchMode(void) { g_mode = MODE_MANUAL; }
const char *Menu_GetModeName(void) { return "TEST"; }
static Location_DataTypeDef loc;
static GPS_DataTypeDef gps;
Location_DataTypeDef *Location_GetData(void) { return &loc; }
GPS_DataTypeDef *GPS_GetData(void) { return &gps; }
uint16_t Lidar_GetMinDistance(uint16_t c, uint16_t w) { (void)c; (void)w; return 800; }
float SensorADC_GetVoltage(int c) { (void)c; return 12.0f; }

FRESULT f_open(FIL *f, const char *p, uint8_t m) { (void)f; (void)p; (void)m; return FR_NOT_READY; }
FRESULT f_read(FIL *f, void *b, UINT n, UINT *r) { return FR_NOT_READY; }
FRESULT f_write(FIL *f, const void *b, UINT n, UINT *w) { return FR_NOT_READY; }
FRESULT f_lseek(FIL *f, uint32_t p) { return FR_NOT_READY; }
FRESULT f_close(FIL *f) { return FR_OK; }
FRESULT f_mkdir(const char *p) { return FR_OK; }

#include "../Mid/Src/map_collent.c"
#include "../Program/Source/uart_host.c"

static void feed(const char *s)
{
    size_t n = strlen(s);
    memcpy(rx, s, n);
    rx[n] = 0;
    rxRead = 0;
    UARTHost_Process(1000);
}
int main(void)
{
    UARTHost_Init();
    Command_Exec("manual", "BT");
    feed("F");
    CHECK(g_mode == MODE_MANUAL && lastDirection == CAR_DIR_FORWARD && moveCalls == 1);
    feed("{\"cmd\": \"speed\", \"v\": 0.45}\r\n");
    CHECK(g_speed == 45);
    feed("{\"cmd\": \"teach\", \"act\": \"add\", \"type\": \"PARK\", \"x\": 1.5, \"y\": 2.5}\r\n");
    CHECK(MapCollent_GetCount() == 1);
    CHECK(MapCollent_GetNode(0)->x == 150 && MapCollent_GetNode(0)->y == 250);
    feed("{\"cmd\": \"voice\", \"text\": \"start\"}\r\n");
    CHECK(g_mode == MODE_PATROL);
    feed("{\"cmd\": \"estop\"}\n");
    CHECK(stopped);
    feed("F");
    CHECK(moveCalls == 1);
    feed("{\"cmd\": \"reset\"}\n");
    CHECK(g_mode == MODE_IDLE && !stopped);
    feed("{\"cmd\": \"ask\", \"what\": \"map\"}\n");
    CHECK(strstr((char *)tx, "MAP_META") != 0);
    printf("UART RESULT: %s\n", failed ? "FAIL" : "PASS");
    return failed ? 1 : 0;
}
