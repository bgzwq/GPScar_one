#ifndef __MENU_H
#define __MENU_H

#include "stm32f4xx.h"
#include "mpu6050.h"

typedef enum
{
    MODE_IDLE = 0,
    MODE_MANUAL,
    MODE_PATROL,
    MODE_SCAN,
    MODE_APPROACH,
    MODE_ALIGN,
    MODE_WAIT_ARM,
    MODE_UNLOAD,
    MODE_INSPECT,
    MODE_RETURN,
    MODE_ESTOP,
    MODE_AVOID,
    MODE_RADAR,
    MODE_VISION,
    MODE_NUM
} WorkMode_TypeDef;

extern WorkMode_TypeDef g_mode;
extern int16_t g_speed;
extern uint8_t g_mpuOK;
extern uint8_t g_basket;
extern float g_distFixed;
extern float g_distRadar;
extern float g_servoAngle;
extern MPU6050_DataTypeDef g_mpu;

void App_SwitchMode(void);
void App_SetMode(WorkMode_TypeDef mode);
void Menu_SplashScreen(void);
void Menu_KeyProcess(void);
void Menu_Refresh(void);
const char *Menu_GetModeName(void);
void Menu_MapSave(void);

#endif
