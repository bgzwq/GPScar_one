#ifndef __LANYACON_H
#define __LANYACON_H

#include "stm32f4xx.h"

/*---------------------- 蓝牙遥控协议(4字节帧) ------------------*/
/*
 * 帧头: 0xA5
 * X轴(0~255): 0x00=左极限, 0xFF=右极限
 * Y轴(0~255): 0x00=上极限(前进), 0xFF=下极限(后退)
 * 按键: 0x00=未按下, 0x01=急停(停车并翻转LED2/3)
 *       0x02=PF13输出高电平300ms(非阻塞, 到点自动恢复低电平)
 *       0x03=PF9输出低电平300ms(非阻塞, 到点自动恢复高电平)
 *       0x04=PF7输出高电平300ms(非阻塞, 到点自动恢复低电平)
 *
 * 蓝牙模块连接上后持续发送遥控帧即进入蓝牙控制(手动模式下生效);
 * 超过 LANYA_TIMEOUT_MS 没有新帧则自动停车(掉线保护)
 */
#define LANYA_FRAME_HEAD	0xA5
#define LANYA_CENTER		128		// 摇杆中值
#define LANYA_DEADZONE		15		// 摇杆死区(中值附近视为松开)
#define LANYA_SPEED_MAX		60		// 满摇杆对应的目标速度(脉冲/20ms)
#define LANYA_TIMEOUT_MS	500		// 掉线保护超时
#define LANYA_BTN_DEBOUNCE_MS 50	// 按键消抖: 原始状态稳定该时长才确认

/* 运动平滑: 轮速按斜坡逼近目标值, 避免突变打滑 */
#define LANYA_RAMP_MS		20		// 斜坡步进周期
#define LANYA_RAMP_STEP		4		// 每步速度增量(脉冲/20ms), 0->满档约300ms

/* 转弯增益: 轮速差 = 摇杆横移量 × NUM/DEN。
 * 3/4 比原来的 1/2 转弯更急; 摇杆纯左/纯右(前后回中)时原地转弯 */
#define LANYA_TURN_NUM		3
#define LANYA_TURN_DEN		4

/* 按键数据位取值 */
#define LANYA_BTN_NONE		0x00	// 未按下
#define LANYA_BTN_STOP		0x01	// 急停
#define LANYA_BTN_PF13_HIGH	0x02	// PF13高电平脉冲
#define LANYA_BTN_PF9_LOW	0x03	// PF9低电平脉冲
#define LANYA_BTN_PF7_HIGH	0x04	// PF7高电平脉冲

/* PF13/PF9/PF7电平脉冲持续时间(ms), 由LanyaCon_Task()非阻塞计时 */
#define LANYA_PULSE_MS		300

/* 功能键输出口: PF13/PF7(平时低电平) PF9(平时高电平) */
#define LANYA_PF13_PIN		GPIO_Pin_13
#define LANYA_PF9_PIN		GPIO_Pin_9
#define LANYA_PF7_PIN		GPIO_Pin_7
#define LANYA_PULSE_PORT	GPIOF
#define LANYA_PULSE_CLK		RCC_AHB1Periph_GPIOF

/*---------------------- 遥控数据 ----------------------------*/
typedef struct
{
	uint8_t x;			// 摇杆X
	uint8_t y;			// 摇杆Y
	uint8_t button;		// 按键数据位原始值(LANYA_BTN_*)
	uint8_t online;		// 1=遥控在线(最近500ms内有完整帧)
} LanyaCon_DataTypeDef;

/*---------------------- 函数声明 ----------------------------*/

void  LanyaCon_Init(void);				// 功能键输出口(PF9/PF13)初始化
uint8_t LanyaCon_Feed(uint8_t byte);			// 喂入蓝牙收到的1字节, 返回1=属于遥控帧(已消费)
LanyaCon_DataTypeDef *LanyaCon_GetData(void);	// 获取最新遥控数据
void LanyaCon_Task(uint32_t now);				// 主循环调用: 掉线检测 + 电平脉冲到点恢复 + 轮速斜坡

#endif /* __LANYACON_H */
