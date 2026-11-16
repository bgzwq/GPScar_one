#ifndef UI_STUBS_H
#define UI_STUBS_H
#include <stdint.h>
#include <stddef.h>
#define NULL ((void *)0)
typedef struct { int dummy; } GPIO_TypeDef;
typedef struct { int dummy; } TaskHandle_t;
typedef struct { int height; } FONT;
typedef FONT pFONT;
extern FONT ASCII_Font12, ASCII_Font16, ASCII_Font20;
#define LCD_BLACK 0
#define LCD_WHITE 1
#define LCD_CYAN 2
#define LCD_YELLOW 3
#define LCD_RED 4
#define LIGHT_GREEN 5
#define LIGHT_GREY 6
#define LIGHT_CYAN 7
#define LIGHT_YELLOW 8
#define LIGHT_RED 9
#define LIGHT_MAGENTA 10
#define Fill_Space 1
void LCD_ShowNumMode(int);
void LCD_SetBackColor(uint32_t);
void LCD_Clear(void);
void LCD_SetAsciiFont(pFONT *);
void LCD_SetColor(uint32_t);
void LCD_DisplayString(uint16_t, uint16_t, char *);
void LCD_DisplayNumber(uint16_t, uint16_t, int32_t, uint8_t);
void LCD_DisplayDecimals(uint16_t, uint16_t, float, uint8_t, uint8_t);
void LCD_ClearRect(uint16_t, uint16_t, uint16_t, uint16_t);

typedef enum { MODE_IDLE, MODE_MANUAL, MODE_PATROL, MODE_SCAN, MODE_APPROACH, MODE_ALIGN, MODE_WAIT_ARM, MODE_UNLOAD, MODE_INSPECT, MODE_RETURN, MODE_ESTOP, MODE_AVOID, MODE_RADAR, MODE_VISION, MODE_NUM } WorkMode_TypeDef;
extern WorkMode_TypeDef g_mode;
extern int16_t g_speed;
extern uint8_t g_mpuOK, g_basket;
extern float g_distFixed, g_distRadar, g_servoAngle;
typedef struct { float accXms, accYms, accZms, gyroXds, gyroYds, gyroZds, temperature; } MPU6050_DataTypeDef;
extern MPU6050_DataTypeDef g_mpu;
void App_SwitchMode(void);
const char *Menu_GetModeName(void);
void Menu_SplashScreen(void);
void Menu_Refresh(void);
void Menu_KeyProcess(void);
void Menu_MapSave(void);

#define MOTOR_NUM 4
#define GPIOE ((GPIO_TypeDef *)1)
#define GPIOG ((GPIO_TypeDef *)2)
#define GPIOB ((GPIO_TypeDef *)3)
#define M1_IN1_PORT GPIOE
#define M1_IN2_PORT GPIOE
#define M2_IN1_PORT GPIOE
#define M2_IN2_PORT GPIOE
#define M3_IN1_PORT GPIOG
#define M3_IN2_PORT GPIOG
#define M4_IN1_PORT GPIOB
#define M4_IN2_PORT GPIOB
#define M1_IN1_PIN 1
#define M1_IN2_PIN 2
#define M2_IN1_PIN 3
#define M2_IN2_PIN 4
#define M3_IN1_PIN 5
#define M3_IN2_PIN 6
#define M4_IN1_PIN 7
#define M4_IN2_PIN 8
#define LOC_ACC_AXIS 0
#define ADC_CH_POWER1 0
#define ADC_CH_POWER2 1
#define ADC_CH_LIGHT 2
#define ADC_CH_HUMIDITY 3
#define CAR_KEY_EVENT_NONE 0
#define CAR_KEY_EVENT_PRESS 1
#define CAR_KEY_EVENT_LONG 2
#define CAR_KEY1 1
#define CAR_KEY2 2
#define CAR_KEY3 3
#define CAR_KEY4 4
#define CAR_KEY_NUM 4
#define CAR_LED_NUM 4
#define LED1_PORT GPIOE
#define LED1_PIN 1
#define NAVLOG_CSV_PATH "0:/CARLOG/TRACK.CSV"
#define NAVLOG_BMP_PATH "0:/CARLOG/MAP.BMP"

typedef struct { float latitude, longitude, speedKmh, course, heading; int32_t latE7, lonE7; uint8_t satellites, source; } Location_DataTypeDef;
typedef struct { float latitude, longitude, hdop; uint8_t hour, minute, second, fixQuality; } GPS_DataTypeDef;
typedef struct { uint32_t rxLines, rxCommands, rxErrors, txTelemetry, lastRxMs; uint8_t wifi, bluetooth, voice; char lastCommand[32], lastRx[64], lastTx[80]; } UARTHost_Status_TypeDef;
typedef struct { uint8_t x, y, button, online; } LanyaCon_DataTypeDef;
typedef struct { uint32_t rxFrames, rxBadFrames, txFrames, overflow, lastFrameMs; uint8_t online, grabPending; } K230_Status_TypeDef;
typedef struct { uint16_t distanceMm[360]; uint32_t packets, crcErrors, lastPacketMs; uint16_t speed; uint8_t online; } Lidar_DataTypeDef;
typedef struct { uint16_t id; uint8_t type; } minimap;
typedef unsigned int UINT;
typedef uint32_t DWORD;
typedef struct { DWORD fsize; } FILINFO;
typedef struct { uint8_t csize; } FATFS;
typedef enum { FR_OK, FR_DISK_ERR } FRESULT;

int GPIO_ReadOutputDataBit(GPIO_TypeDef *, uint16_t);
int GPIO_ReadInputDataBit(GPIO_TypeDef *, uint16_t);
void Delay_ms(uint32_t);
uint32_t Get_Ms(void);
void Buzzer_Beep(uint32_t, uint32_t);
void Buzzer_BeepAsync(uint32_t, uint32_t);
void CarKey_Poll(void);
uint8_t CarKey_GetEvent(uint8_t *);
uint8_t CarKey_GetState(uint8_t);
uint8_t CarLED_Get(uint8_t);
int16_t Car_GetTarget(uint8_t);
int16_t Car_GetSpeed(uint8_t);
uint8_t Car_IsRunning(void);
int16_t Motor_GetPwm(uint8_t);
int32_t Encoder_GetTotal(uint8_t);
int16_t Encoder_ReadDelta(uint8_t);
void Encoder_ClearTotal(uint8_t);
void Motor_SetSpeed(uint8_t, int16_t);
void Motor_StopAll(void);
void Motor_Stop(uint8_t);
void Car_Stop(void);
float SensorADC_GetVoltage(int);
uint16_t SensorADC_GetRaw(int);
Location_DataTypeDef *Location_GetData(void);
GPS_DataTypeDef *GPS_GetData(void);
const char *Location_GetSourceName(void);
uint8_t Location_IsCalibrated(void);
float Location_GetAccBias(void);
float Location_GetGyroBias(void);
float Location_GetGyroNoise(void);
uint32_t GPS_GetRmcCount(void);
uint32_t GPS_GetQueueDropped(void);
uint32_t NavLog_GetCount(void);
uint32_t NavLog_GetRejectCount(void);
float NavLog_GetDistanceM(void);
const char *NavLog_GetStateName(void);
uint8_t NavLog_IsSdOk(void);
void NavLog_Clear(void);
void NavLog_Sync(void);
void MapView_ZoomOut(void);
void MapView_ZoomIn(void);
void MapView_NotifyCleared(void);
void MapView_Refresh(uint8_t);
uint8_t MapView_SaveToSd(void);
const char *K230_GetStateName(void);
K230_Status_TypeDef *K230_GetStatus(void);
UARTHost_Status_TypeDef *UARTHost_GetStatus(void);
Lidar_DataTypeDef *Lidar_GetData(void);
uint16_t Lidar_GetMinDistance(uint16_t, uint16_t);
uint8_t Estop_IsActive(void);
uint8_t Estop_GetSource(void);
uint32_t Estop_GetCount(void);
uint16_t MapCollent_GetCount(void);
int map_write_SD(minimap *, char *);
LanyaCon_DataTypeDef *LanyaCon_GetData(void);
FRESULT f_stat(const char *, FILINFO *);
FRESULT f_getfree(const char *, DWORD *, FATFS **);
size_t xPortGetFreeHeapSize(void);
size_t xPortGetMinimumEverFreeHeapSize(void);
uint32_t uxTaskGetStackHighWaterMark(void *);
#endif
