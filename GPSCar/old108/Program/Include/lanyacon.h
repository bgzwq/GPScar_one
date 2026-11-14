#ifndef __LANYACON_H
#define __LANYACON_H

#include "stm32f4xx.h"

/*---------------------- 蓝牙遥控协议(4字节帧) ------------------*/
/*
 * 帧头: 0xA5
 * X轴(0~255): 0x00=左极限, 0xFF=右极限
 * Y轴(0~255): 0x00=上极限(前进), 0xFF=下极限(后退)
 * 按键: 0x00=未按下, 0x01=按下(急停)
 *
 * 蓝牙模块连接上后持续发送遥控帧即进入蓝牙控制(手动模式下生效);
 * 超过 LANYA_TIMEOUT_MS 没有新帧则自动停车(掉线保护)
 */
#define LANYA_FRAME_HEAD	0xA5
#define LANYA_CENTER		128		// 摇杆中值
#define LANYA_DEADZONE		15		// 摇杆死区(中值附近视为松开)
#define LANYA_SPEED_MAX		60		// 满摇杆对应的目标速度(脉冲/20ms)
#define LANYA_TIMEOUT_MS	500		// 掉线保护超时
#define LANYA_BTN_DEBOUNCE_MS 50	// 急停按键消抖: 原始状态稳定该时长才确认

/*---------------------- 遥控数据 ----------------------------*/
typedef struct
{
	uint8_t x;			// 摇杆X
	uint8_t y;			// 摇杆Y
	uint8_t button;		// 1=按下(急停)
	uint8_t online;		// 1=遥控在线(最近500ms内有完整帧)
} LanyaCon_DataTypeDef;

/*---------------------- 函数声明 ----------------------------*/

uint8_t LanyaCon_Feed(uint8_t byte);			// 喂入蓝牙收到的1字节, 返回1=属于遥控帧(已消费)
LanyaCon_DataTypeDef *LanyaCon_GetData(void);	// 获取最新遥控数据
void LanyaCon_Task(uint32_t now);				// 主循环调用: 掉线检测, 超时自动停车

#endif /* __LANYACON_H */
