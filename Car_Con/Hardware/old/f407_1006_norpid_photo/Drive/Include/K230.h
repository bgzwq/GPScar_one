#ifndef _K230_H
#define _K230_H
#include <stdint.h>

/*=================================================================================================
 * K230 视觉模块通信协议
 *-------------------------------------------------------------------------------------------------
 * 物理层: USART1, PA9(TX)/PA10(RX), 115200 8N1
 *   注: 工程已移除全部printf调试输出, USART1上只有本协议的帧, 没有杂散文本。
 *
 * 方向1: K230 -> 主控  固定8字节, 无校验(与原固件保持一致, 不要改K230端)
 *   0x55 | LR | X高 | X低 | UD | Y高 | Y低 | flag
 *     LR   = 1目标在左侧  0目标在右侧
 *     UD   = 1目标在上侧  0目标在下侧
 *     X/Y  = 目标与画面中心的距离(像素)
 *     flag = 1请求夹取     0继续前行
 *
 * 方向2: 主控 -> K230  变长, 带校验
 *   0xAA | CMD | LEN | payload[LEN] | CHK
 *     CHK = 从0xAA开始到payload最后一字节逐字节异或
 *   指令见下面 K230_CMD_* 定义; 无payload时 LEN=0
 *===============================================================================================*/

/*---------------------- K230 -> 主控 ----------------------------*/
typedef struct K230_time
{
	uint8_t head; //帧头01010101即0x55
    uint8_t LR;//判断是左还是右侧，1是左侧，0是右侧
    uint16_t x;//x值与屏幕的距离
    uint8_t UD;//判断是上还是下侧，1是上侧，0是下侧
    uint16_t y;//y值与屏幕的距离
    uint8_t flag;//1是要夹取，0是继续前行
}K230_time;

typedef struct K230_con
{
	uint8_t speed1; 
    uint8_t speed2;
    uint8_t speed3;
    uint8_t speed4;
	uint8_t turn; 
    uint8_t flag;//1是数据正常，
}K230_con;

/*---------------------- 主控 -> K230 指令 ------------------------*/
#define K230_TX_HEAD		0xAA	// 下行帧头
#define K230_TX_MAX_PAYLOAD	8		// 单帧最大负载

#define K230_CMD_TRACK_START	0x01	// 开始视觉追踪(进入VISION模式时下发)
#define K230_CMD_TRACK_STOP		0x02	// 停止追踪(离开VISION模式时下发)
#define K230_CMD_GRAB_ACK		0x03	// 已收到夹取请求, 小车已停车
#define K230_CMD_GRAB_DONE		0x04	// 夹取流程结束, 允许K230继续输出目标
#define K230_CMD_RECALIB		0x05	// 要求K230重新标定/重置目标
#define K230_CMD_HEARTBEAT		0x06	// 主控心跳(1s一次, 供K230判断掉线)
#define K230_CMD_SET_SPEED		0x07	// payload[0]=基础速度(0~100)

/*---------------------- 链路状态(供菜单/串口显示) ----------------*/
typedef struct
{
	uint32_t rxFrames;		// 收到的完整帧数
	uint32_t rxBadFrames;	// 疑似错帧数(flag字段不是0/1)
	uint32_t txFrames;		// 已下发的指令帧数
	uint32_t overflow;		// 接收环形缓冲满而丢弃的字节数
	uint32_t lastFrameMs;	// 最后一帧的时间戳
	uint8_t  online;		// 1=在超时时间内收到过帧
	uint8_t  grabPending;	// 1=已请求夹取, 等机械臂/超时
	K230_time raw;			// 最新原始帧
	K230_con  con;			// 最新解析出的控制量
} K230_Status_TypeDef;

/*---------------------- 函数声明 ----------------------------*/

//初始化串口一，PA9，10为主控板和K230的桥梁
void K230_Init(void);

//主循环调用: 从缓冲区取出K230发过来的信息，并进行解析
void K230_Process(void);

//接收K230发过来的信息，放到缓冲区里面，并进行解析
void K230_sovle(struct K230_time *k230_t,struct K230_con *k230_c);

/* 下发一条指令帧(带校验和), 阻塞时间约为 LEN+4 字节, 很短 */
void K230_SendCmd(uint8_t cmd, const uint8_t *payload, uint8_t len);

/* 进入/离开视觉模式时调用: 通知K230开始或停止输出目标 */
void K230_StartTrack(void);
void K230_StopTrack(void);

/* 机械臂动作完成后调用: 清除夹取挂起状态并通知K230继续 */
void K230_GrabDone(void);

/* 机械臂动作挂起时由主循环调用, 超时自动收尾(没有机械臂时保证流程不卡住) */
void K230_GrabTask(void);

uint8_t  K230_IsOnline(void);				// K230当前是否在线
const char *K230_GetStateName(void);		// 链路状态名 "ON  "/"OFF "/"WAIT"
K230_Status_TypeDef *K230_GetStatus(void);	// 链路统计

//测试缓冲区(定义在K230.c, 头文件只声明)
extern uint8_t K230_test[400];

#endif
