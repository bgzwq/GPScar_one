#ifndef __MENU_H
#define __MENU_H

#include "stm32f4xx.h"
#include "mpu6050.h"

/*---------------------- 工作模式 ----------------------------*/
/*
 * 【关于 MODE_VISION】
 *   原来 K230 是在 MODE_MANUAL 下直接驱动电机的, 而蓝牙摇杆(lanyacon.c)和
 *   语音(asrpro)在手动模式下同样会写电机速度, 两个控制源会互相覆盖轮速。
 *   现在把视觉追踪独立成 MODE_VISION: 只有该模式下 K230 才接管底盘,
 *   手动模式只受摇杆/语音控制, 彻底避免抢控制权。
 *   进入/离开该模式时会由 App_SwitchMode() 通知 K230 开始/停止输出目标。
 */
typedef enum
{
	MODE_MANUAL = 0,	// 手动遥控(按键/蓝牙/语音)
	MODE_AVOID,			// 自动避障
	MODE_RADAR,			// 雷达扫描演示
	MODE_VISION,		// K230视觉追踪
	MODE_NUM
} WorkMode_TypeDef;

/*---------------------- 共享运行状态(变量定义在 main.c) --------*/
extern WorkMode_TypeDef g_mode;			// 当前工作模式
extern int16_t  g_speed;				// 目标速度(编码器脉冲/20ms)
extern uint8_t  g_mpuOK;				// MPU6050是否在线
extern float    g_distFixed;			// 车头固定超声波距离(cm)
extern float    g_distRadar;			// 雷达超声波距离(cm)
extern float    g_servoAngle;			// 当前舵机角度
extern MPU6050_DataTypeDef g_mpu;		// MPU6050最新数据

/*---------------------- 由 main.c 提供的接口 -------------------*/
void App_SwitchMode(void);				// 循环切换工作模式(按键/蓝牙/语音共用)

/*---------------------- 菜单 / 人机交互接口 --------------------*/
void        Menu_SplashScreen(void);	// LCD开机画面 + 开机提示音
void        Menu_KeyProcess(void);		// 按键扫描处理(50ms周期调用)
void        Menu_Refresh(void);			// LCD主界面刷新(300ms周期调用)
const char *Menu_GetModeName(void);		// 当前模式名(LCD页眉显示用)
void        Menu_MapSave(void);			// 保存地图到SD卡(地图页KEY4)

#endif /* __MENU_H */
