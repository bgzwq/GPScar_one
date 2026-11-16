#include <stdio.h>
#include <string.h>
#include "ui_stubs.h"

FONT ASCII_Font12, ASCII_Font16, ASCII_Font20;
WorkMode_TypeDef g_mode = MODE_IDLE;
int16_t g_speed = 30;
uint8_t g_mpuOK = 1, g_basket = 3;
float g_distFixed = 80, g_distRadar = 100, g_servoAngle = 90;
MPU6050_DataTypeDef g_mpu;
static char drawn[512][80];
static int drawCount, clearCount, failed;
static uint8_t keyEvent, keyId;
#define CHECK(c) do { if (c) printf("PASS %s\n", #c); else { printf("FAIL %d %s\n", __LINE__, #c); failed++; } } while (0)
void LCD_ShowNumMode(int m) {(void)m;}
void LCD_SetBackColor(uint32_t c) {(void)c;}
void LCD_Clear(void) {clearCount++;}
void LCD_SetAsciiFont(pFONT *f) {(void)f;}
void LCD_SetColor(uint32_t c) {(void)c;}
void LCD_DisplayString(uint16_t x, uint16_t y, char *s) {(void)x;(void)y;if(drawCount<512){strncpy(drawn[drawCount],s,79);drawn[drawCount][79]=0;}drawCount++;}
void LCD_DisplayNumber(uint16_t x,uint16_t y,int32_t n,uint8_t l){char b[32];(void)x;(void)y;(void)l;snprintf(b,sizeof(b),"%ld",(long)n);LCD_DisplayString(0,0,b);}
void LCD_DisplayDecimals(uint16_t x,uint16_t y,float n,uint8_t l,uint8_t d){char b[32];(void)x;(void)y;(void)l;(void)d;snprintf(b,sizeof(b),"%.2f",n);LCD_DisplayString(0,0,b);}
void LCD_ClearRect(uint16_t x,uint16_t y,uint16_t w,uint16_t h){(void)x;(void)y;(void)w;(void)h;}
static int shown(const char *s){int i;for(i=0;i<drawCount&&i<512;i++)if(strstr(drawn[i],s))return 1;return 0;}
void Delay_ms(uint32_t n){(void)n;}
uint32_t Get_Ms(void){return 1000;}
void Buzzer_Beep(uint32_t a,uint32_t b){(void)a;(void)b;}
void Buzzer_BeepAsync(uint32_t a,uint32_t b){(void)a;(void)b;}
void CarKey_Poll(void){}
uint8_t CarKey_GetEvent(uint8_t *k){uint8_t e=keyEvent;if(e&&k)*k=keyId;keyEvent=0;return e;}
uint8_t CarKey_GetState(uint8_t k){(void)k;return 0;}
uint8_t CarLED_Get(uint8_t k){(void)k;return 0;}
int GPIO_ReadOutputDataBit(GPIO_TypeDef*p,uint16_t n){(void)p;(void)n;return 0;}
int GPIO_ReadInputDataBit(GPIO_TypeDef*p,uint16_t n){(void)p;(void)n;return 0;}
int16_t Car_GetTarget(uint8_t i){(void)i;return 30;}
int16_t Car_GetSpeed(uint8_t i){(void)i;return 28;}
uint8_t Car_IsRunning(void){return 0;}
int16_t Motor_GetPwm(uint8_t i){(void)i;return 480;}
int32_t Encoder_GetTotal(uint8_t i){(void)i;return 123;}
int16_t Encoder_ReadDelta(uint8_t i){(void)i;return 1;}
void Encoder_ClearTotal(uint8_t i){(void)i;}
void Motor_SetSpeed(uint8_t i,int16_t s){(void)i;(void)s;}
void Motor_StopAll(void){}
void Motor_Stop(uint8_t i){(void)i;}
void Car_Stop(void){}
void App_SwitchMode(void){g_mode=MODE_MANUAL;}
float SensorADC_GetVoltage(int c){(void)c;return 12.2f;}
uint16_t SensorADC_GetRaw(int c){(void)c;return 2048;}
static Location_DataTypeDef loc={25.0f,113.0f,1.0f,90,90,0,0,8,0};
static GPS_DataTypeDef gps={25.0f,113.0f,1.2f,12,0,0,1};
Location_DataTypeDef *Location_GetData(void){return &loc;}
GPS_DataTypeDef *GPS_GetData(void){return &gps;}
const char *Location_GetSourceName(void){return "GPS ";}
uint8_t Location_IsCalibrated(void){return 1;}
float Location_GetAccBias(void){return 0;}
float Location_GetGyroBias(void){return 0;}
float Location_GetGyroNoise(void){return 0.1f;}
uint32_t GPS_GetRmcCount(void){return 20;}
uint32_t GPS_GetQueueDropped(void){return 0;}
uint32_t NavLog_GetCount(void){return 16;}
uint32_t NavLog_GetRejectCount(void){return 2;}
float NavLog_GetDistanceM(void){return 25.5f;}
const char *NavLog_GetStateName(void){return "OK  ";}
uint8_t NavLog_IsSdOk(void){return 1;}
void NavLog_Clear(void){}
void NavLog_Sync(void){}
void MapView_ZoomOut(void){}
void MapView_ZoomIn(void){}
void MapView_NotifyCleared(void){}
void MapView_Refresh(uint8_t v){(void)v;}
uint8_t MapView_SaveToSd(void){return 1;}
const char *K230_GetStateName(void){return "ON  ";}
static K230_Status_TypeDef k230;
K230_Status_TypeDef *K230_GetStatus(void){return &k230;}
static UARTHost_Status_TypeDef host={10,4,0,20,0,1,0,1,"start","{\"cmd\":\"start\"}","{\"t\":1000}"};
UARTHost_Status_TypeDef *UARTHost_GetStatus(void){return &host;}
static Lidar_DataTypeDef lidar;
Lidar_DataTypeDef *Lidar_GetData(void){return &lidar;}
uint16_t Lidar_GetMinDistance(uint16_t c,uint16_t w){(void)c;(void)w;return 800;}
uint8_t Estop_IsActive(void){return 0;}
uint8_t Estop_GetSource(void){return 0;}
uint32_t Estop_GetCount(void){return 0;}
uint16_t MapCollent_GetCount(void){return 4;}
int map_write_SD(minimap*m,char*p){(void)m;(void)p;return 1;}
static LanyaCon_DataTypeDef joy;
LanyaCon_DataTypeDef *LanyaCon_GetData(void){return &joy;}
FRESULT f_stat(const char*p,FILINFO*i){(void)p;i->fsize=1234;return FR_OK;}
FRESULT f_getfree(const char*p,DWORD*c,FATFS**f){static FATFS fs={8};(void)p;*c=2048;*f=&fs;return FR_OK;}
size_t xPortGetFreeHeapSize(void){return 12000;}
size_t xPortGetMinimumEverFreeHeapSize(void){return 10000;}
uint32_t uxTaskGetStackHighWaterMark(void*t){(void)t;return 300;}

#include "../Mid/Src/menu.c"

int main(void)
{
    static const char *markers[PAGE_NUM]={"Smart Picking Car","Last RX:","T:target","Light/Humi","Zero:","RMC:","heartbeat","Pts:","TRACK.CSV","Heap:"};
    int page, before;
    Menu_SplashScreen();
    CHECK(shown("Smart Car"));
    before=clearCount;
    Menu_Refresh();
    CHECK(clearCount==before);
    CHECK(shown("Smart Picking Car")&&shown("STATUS: READY"));
    for(page=0;page<PAGE_NUM;page++)
    {
        s_page=(uint8_t)page;
        before=drawCount;
        Menu_Refresh();
        CHECK(drawCount>before);
        CHECK(shown(markers[page]));
    }
    s_page=PAGE_OVERVIEW;
    keyEvent=CAR_KEY_EVENT_PRESS;keyId=CAR_KEY2;
    Menu_KeyProcess();
    CHECK(s_page==PAGE_COMM);
    Menu_Refresh();
    CHECK(shown(host.lastRx)&&shown("Last TX:"));
    printf("MENU RESULT: %s (%d pages)\n",failed?"FAIL":"PASS",PAGE_NUM);
    return failed?1:0;
}
