/***
	*************************************************************************************************
	*	@version V1.0
	*	@brief   四驱底盘速度闭环控制
   *************************************************************************************************
   *  @description
	*
>>>>> 文件说明：
	*
	*  1. 组合 motor.c 与 encoder.c, 4轮开环PWM直驱 (速度PID闭环已注释, 需要时取消注释恢复)
	*  2. Car_PID_Update() 已注释, 原需固定每 20ms 调用一次
	*  3. 目标/反馈速度单位: 编码器脉冲数 / 20ms (开环时按 CAR_OPENLOOP_SCALE 直接映射PWM)
	*  4. 车轮布局: M1=左前 M2=右前 M3=左后 M4=右后
	*
	************************************************************************************************
***/

#include "car.h"
#include "motor.h"
#include "encoder.h"
#include "estop.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct
{
	float kp, ki, kd;
	float integral;
	float lastErr;
	int16_t target;		// 目标速度
	int16_t speed;		// 反馈速度
	uint8_t motorId;
	uint8_t encoderId;
} Wheel_TypeDef;

static Wheel_TypeDef s_wheel[MOTOR_NUM];
static uint8_t s_running = 0;
/*---------------------- 左右轮编号表 ----------------------------*/
/*
 * 【2026-11 修正】以前"哪几个轮子算左侧"只写在注释里, 而且本文件顶部的注释
 * 写的是 "M2=右前 M3=左后", 与 motor.h / car.h 的 "M1M2=左、M3M4=右" 互相矛盾。
 * 现在把左右分配做成表, 唯一来源是 car.h 里的 CAR_LEFT_MOTORS / CAR_RIGHT_MOTORS,
 * 以后要改线序只改那两个宏。
 */
static const uint8_t s_leftWheel[2]  = CAR_LEFT_MOTORS;
static const uint8_t s_rightWheel[2] = CAR_RIGHT_MOTORS;

/*---------------------- 底盘命令邮箱 ----------------------------*/
/* kind: 1=方向 2=差速转弯 3=左右两侧 4=四个轮子 5=单轮目标速度 6=单轮原始PWM 7=四轮制动 */
typedef struct
{
	uint8_t kind;
	uint8_t src;		// CAR_SRC_*
	uint8_t id;			// 单轮命令用的轮号
	uint8_t dir;		// CarDir_TypeDef
	int16_t v[4];		// 参数(1个或4个)
	uint8_t seq;		// 序号: ControlTask 用它判断"有没有新命令"
} CarPost_TypeDef;

static volatile CarPost_TypeDef s_post;	// 生产者: 任意任务(临界区内写)
static uint8_t s_applied;				// 已经执行到的序号
static uint8_t s_testLock;				// 1=自检独占底盘
static uint8_t s_activeSrc;				// 最近一次真正生效的命令源

/*************************************************************************************************
*	函 数 名:	Car_Init
*
*	函数功能:	初始化电机、编码器与PID参数
*
*************************************************************************************************/
void Car_Init(void)
{
	uint8_t i;

	Motor_Init();
	Encoder_Init();

	for (i = 0; i < MOTOR_NUM; i++)
	{
		/* PID参数初始化已注释(开环直驱)
		s_wheel[i].kp        = CAR_PID_KP;
		s_wheel[i].ki        = CAR_PID_KI;
		s_wheel[i].kd        = CAR_PID_KD;
		s_wheel[i].integral  = 0;
		s_wheel[i].lastErr   = 0;
		*/
		s_wheel[i].target    = 0;
		s_wheel[i].speed     = 0;
		s_wheel[i].motorId   = i;
		s_wheel[i].encoderId = i;	// 电机与编码器一一对应
	}
	s_running = 0;
}

/*************************************************************************************************
*	函 数 名:	Car_SetWheelSpeed
*
*	入口参数:	id     - 轮号 0~3
*				target - 目标速度(编码器脉冲数/20ms), 负值为反转
*
*	函数功能:	设置单轮速度, PID已注释, 开环直接按 CAR_OPENLOOP_SCALE 映射输出PWM
*
*************************************************************************************************/
void Car_SetWheelSpeed(uint8_t id, int16_t target)
{
	if (id >= MOTOR_NUM) return;
	s_wheel[id].target = target;
	s_running = 1;

	Motor_SetSpeed(id, (int16_t)(target * CAR_OPENLOOP_SCALE));	// 开环直驱, 不再经过PID
}

/*************************************************************************************************
*	函 数 名:	Car_Move
*
*	入口参数:	dir   - 运动方向
*				speed - 目标速度(脉冲/20ms), 取绝对值
*
*	函数功能:	整车运动控制, 原地转向时左右轮反向
*
*************************************************************************************************/
void Car_Move(CarDir_TypeDef dir, int16_t speed)
{
	int16_t left = 0, right = 0;

	if (speed < 0) speed = -speed;

	switch (dir)
	{
		case CAR_DIR_FORWARD:  left =  speed; right =  speed; break;
		case CAR_DIR_BACKWARD: left = -speed; right = -speed; break;
		case CAR_DIR_LEFT:     left = -speed; right =  speed; break;	// 左轮倒转右轮正转 -> 原地左转
		case CAR_DIR_RIGHT:    left =  speed; right = -speed; break;
		case CAR_DIR_STOP:
		default:
			Car_Stop();
			return;
	}

	/* M1 M2 左侧, M3 M4 右侧 */
	/* 左侧 = s_leftWheel[0..1], 右侧 = s_rightWheel[0..1] (见 car.h 的宏) */
	Car_SetWheelSpeed(s_leftWheel[0],  left);
	Car_SetWheelSpeed(s_leftWheel[1],  left);
	Car_SetWheelSpeed(s_rightWheel[0], right);
	Car_SetWheelSpeed(s_rightWheel[1], right);
}

/*************************************************************************************************
*	函 数 名:	Car_Turn
*
*	入口参数:	dir   - CAR_DIR_LEFT / CAR_DIR_RIGHT
*				speed - 静止时原地转向的目标速度
*
*	函数功能:	转弯控制。行进中(左右轮同向且在动)做差速转弯: 外侧轮保持当前速度,
*				内侧轮降到外侧的 CAR_TURN_INNER_PCT%, 实现边行进边转弯, 不再原地自旋;
*				静止或正在原地转时退化为 Car_Move 原地转向。
*				再次同向转弯结果不变(始终以较快一侧为基准), 反向转弯则切到另一侧。
*
*************************************************************************************************/
void Car_Turn(CarDir_TypeDef dir, int16_t speed)
{
	int16_t lt, rt, al, ar, base, inner, outer, sign;

	lt = s_wheel[MOTOR_1].target;	// 左侧当前目标速度
	rt = s_wheel[MOTOR_3].target;	// 右侧当前目标速度

	/* 静止 / 正在原地转(左右异号) / 仅单侧在动: 直接原地转向 */
	if (!s_running || lt == 0 || rt == 0 || ((lt < 0) != (rt < 0)))
	{
		Car_Move(dir, speed);
		return;
	}

	/* 以较快一侧为外侧基准, 保持整体前进/后退方向不变 */
	al   = (lt >= 0) ? lt : -lt;
	ar   = (rt >= 0) ? rt : -rt;
	base = (al > ar) ? al : ar;
	sign = (lt > 0) ? 1 : -1;

	outer = (int16_t)(base * sign);
	inner = (int16_t)(base * CAR_TURN_INNER_PCT / 100 * sign);

	if (dir == CAR_DIR_LEFT)	// 左转: 左侧为内侧
	{
		Car_SetWheelSpeed(MOTOR_1, inner);
		Car_SetWheelSpeed(MOTOR_2, inner);
		Car_SetWheelSpeed(MOTOR_3, outer);
		Car_SetWheelSpeed(MOTOR_4, outer);
	}
	else						// 右转: 右侧为内侧
	{
		Car_SetWheelSpeed(MOTOR_1, outer);
		Car_SetWheelSpeed(MOTOR_2, outer);
		Car_SetWheelSpeed(MOTOR_3, inner);
		Car_SetWheelSpeed(MOTOR_4, inner);
	}
}

/*************************************************************************************************
*	函 数 名:	Car_Stop
*
*	函数功能:	制动全部电机
*
*************************************************************************************************/
void Car_Stop(void)
{
	uint8_t i;

	for (i = 0; i < MOTOR_NUM; i++)
	{
		s_wheel[i].target   = 0;
		/* PID状态清零已注释(开环直驱)
		s_wheel[i].integral = 0;
		s_wheel[i].lastErr  = 0;
		*/
	}
	s_running = 0;
	Motor_StopAll();
	/* 停车同时作废"还没执行"的命令: 否则 ControlTask 下一拍又会把车开起来 */
	s_applied = s_post.seq;
}

/*************************************************************************************************
*	函 数 名:	Car_SpeedUpdate
*
*	函数功能:	只刷新编码器反馈速度(脉冲/20ms), 供菜单显示, 不做PID闭环
*
*	说    明:	需固定每 20ms 调用一次(占用原 Car_PID_Update 的调用节奏),
*				只读编码器增量, 不动电机输出, 开环直驱不受影响。
*
*************************************************************************************************/
void Car_SpeedUpdate(void)
{
	uint8_t i;

	for (i = 0; i < MOTOR_NUM; i++)
	{
		s_wheel[i].speed = Encoder_ReadDelta(i);	// 读取本周期脉冲增量
	}
}

/*************************************************************************************************
*	函 数 名:	Car_PID_Update
*
*	函数功能:	位置式速度PID, 需固定每 20ms 调用一次
*
*	说    明:	已整体注释 —— 蓝牙遥控时PID闭环导致轮胎顿挫明显, 底盘改为开环PWM直驱.
*				如需恢复闭环, 取消本函数、Car_Init/Car_Stop 中被注释的PID状态行、
*				main.c 中20ms调用以及 car.h 中PID参数与函数声明的注释即可.
*
*************************************************************************************************/
//void Car_PID_Update(void)
//{
//	uint8_t i;
//	float err, out;
//
//	for (i = 0; i < MOTOR_NUM; i++)
//	{
//		s_wheel[i].speed = Encoder_ReadDelta(i);	// 读取本周期脉冲增量
//
//		if (!s_running) continue;
//
//		err = (float)s_wheel[i].target - (float)s_wheel[i].speed;
//
//		// 积分项 + 限幅
//		s_wheel[i].integral += err * s_wheel[i].ki;
//		if (s_wheel[i].integral >  CAR_PID_IMAX) s_wheel[i].integral =  CAR_PID_IMAX;
//		if (s_wheel[i].integral < -CAR_PID_IMAX) s_wheel[i].integral = -CAR_PID_IMAX;
//
//		out = err * s_wheel[i].kp
//			+ s_wheel[i].integral
//			+ (err - s_wheel[i].lastErr) * s_wheel[i].kd;
//		s_wheel[i].lastErr = err;
//
//		if (out >  CAR_PID_OUTMAX) out =  CAR_PID_OUTMAX;
//		if (out < -CAR_PID_OUTMAX) out = -CAR_PID_OUTMAX;
//
//		// 目标为0时直接制动, 避免积分爬行
//		if (s_wheel[i].target == 0)
//		{
//			s_wheel[i].integral = 0;
//			Motor_Stop(i);
//		}
//		else
//		{
//			Motor_SetSpeed(i, (int16_t)out);
//		}
//	}
//}

/*************************************************************************************************
*	函 数 名:	Car_GetSpeed / Car_GetTarget / Car_IsRunning
*
*	函数功能:	状态查询
*
*************************************************************************************************/
int16_t Car_GetSpeed(uint8_t id)
{
	if (id >= MOTOR_NUM) return 0;
	return s_wheel[id].speed;
}

int16_t Car_GetTarget(uint8_t id)
{
	if (id >= MOTOR_NUM) return 0;
	return s_wheel[id].target;
}

uint8_t Car_IsRunning(void)
{
	return s_running;
}
/*************************************************************************************************
*	函 数 名:	Car_Push
*
*	函数功能:	把一条底盘命令放进邮箱(内部函数)
*
*	说    明:	临界区只包住"写结构体 + 序号自增", 几十个时钟周期, 不影响中断响应。
*				序号最后写: 消费者看到新序号时, 其它字段一定已经是新的了。
*
*************************************************************************************************/
static void Car_Push(uint8_t kind, uint8_t src, uint8_t id, uint8_t dir,
					 int16_t v0, int16_t v1, int16_t v2, int16_t v3)
{
	if (s_testLock && src != CAR_SRC_TEST) return;	// 自检期间别的源不许抢底盘

	taskENTER_CRITICAL();
	s_post.kind = kind;
	s_post.src  = src;
	s_post.id   = id;
	s_post.dir  = dir;
	s_post.v[0] = v0;
	s_post.v[1] = v1;
	s_post.v[2] = v2;
	s_post.v[3] = v3;
	s_post.seq++;
	taskEXIT_CRITICAL();
}

/*************************************************************************************************
*	函 数 名:	Car_PostDir / Car_PostTurn / Car_PostLR / Car_PostWheels / Car_PostWheel /
*				Car_PostPwm / Car_PostBrake
*
*	函数功能:	【供任意任务调用】投递一条底盘命令, 立即返回, 不碰任何硬件
*
*************************************************************************************************/
void Car_PostDir(uint8_t src, CarDir_TypeDef dir, int16_t speed)
{
	Car_Push(1, src, 0, (uint8_t)dir, speed, 0, 0, 0);
}

void Car_PostTurn(uint8_t src, CarDir_TypeDef dir, int16_t speed)
{
	Car_Push(2, src, 0, (uint8_t)dir, speed, 0, 0, 0);
}

void Car_PostLR(uint8_t src, int16_t left, int16_t right)
{
	Car_Push(3, src, 0, 0, left, right, 0, 0);
}

void Car_PostWheels(uint8_t src, const int16_t w[4])
{
	if (!w) return;
	Car_Push(4, src, 0, 0, w[0], w[1], w[2], w[3]);
}

void Car_PostWheel(uint8_t src, uint8_t id, int16_t speed)
{
	Car_Push(5, src, id, 0, speed, 0, 0, 0);
}

void Car_PostPwm(uint8_t src, uint8_t id, int16_t pwm)
{
	Car_Push(6, src, id, 0, pwm, 0, 0, 0);
}

void Car_PostBrake(uint8_t src)
void Car_PostPwmAll(uint8_t src, int16_t pwm)
{
	Car_Push(8, src, 0, 0, pwm, 0, 0, 0);
}

/*************************************************************************************************
*	函 数 名:	Car_PostBrake
*
*	函数功能:	投递"四轮制动"命令
*
*************************************************************************************************/
void Car_PostBrake(uint8_t src)
{
	Car_Push(7, src, 0, 0, 0, 0, 0, 0);
}

/*************************************************************************************************
*	函 数 名:	Car_TestLock
*
*	入口参数:	on - 1=自检独占底盘(其它源的命令全部丢弃), 0=恢复正常
*
*************************************************************************************************/
void Car_TestLock(uint8_t on)
{
	s_testLock = on ? 1 : 0;
}

uint8_t Car_GetActiveSrc(void)
{
	return s_activeSrc;
}

/*************************************************************************************************
*	函 数 名:	Car_ControlTask
*
*	函数功能:	【全工程唯一写电机的地方】每 20ms 由 ControlTask 调用一次:
*				1) 刷新四轮编码器反馈
*				2) 取邮箱里最新的一条底盘命令并原子地输出到电机
*
*	说    明:	没有新命令时保持上一次输出(不会因为"没命令"而自动停车);
*				要停车请显式调 Car_Stop() 或 Car_PostBrake()。
*				急停激活时一律不接受新命令。
*
*************************************************************************************************/
void Car_ControlTask(void)
{
	uint8_t        i;
	uint8_t        seq;
	CarPost_TypeDef p;

	/* 1) 编码器反馈(原 Car_SpeedUpdate 的内容, 现在只有这一处读编码器) */
	for (i = 0; i < MOTOR_NUM; i++)
	{
		s_wheel[i].speed = Encoder_ReadDelta(i);
	}

	/* 2) 急停激活: 不接受新命令, 并把挂起的命令作废 */
	if (Estop_IsActive())
	{
		s_applied = s_post.seq;
		return;
	}

	/* 3) 取一条新命令 */
	taskENTER_CRITICAL();
	seq       = s_post.seq;
	p.kind    = s_post.kind;
	p.src     = s_post.src;
	p.id      = s_post.id;
	p.dir     = s_post.dir;
	p.v[0]    = s_post.v[0];
	p.v[1]    = s_post.v[1];
	p.v[2]    = s_post.v[2];
	p.v[3]    = s_post.v[3];
	taskEXIT_CRITICAL();

	if (p.kind == 0 || seq == s_applied) return;			/* 没有新命令 */
	s_applied = seq;

	if (s_testLock && p.src != CAR_SRC_TEST) return;	/* 自检独占底盘 */
	s_activeSrc = p.src;

	switch (p.kind)
	{
		case 1:		/* 整车方向 */
			Car_Move((CarDir_TypeDef)p.dir, p.v[0]);
			break;

		case 2:		/* 行进中差速转弯 */
			Car_Turn((CarDir_TypeDef)p.dir, p.v[0]);
			break;

		case 3:		/* 左右两侧速度(摇杆差速) */
			Car_SetWheelSpeed(s_leftWheel[0],  p.v[0]);
			Car_SetWheelSpeed(s_leftWheel[1],  p.v[0]);
			Car_SetWheelSpeed(s_rightWheel[0], p.v[1]);
			Car_SetWheelSpeed(s_rightWheel[1], p.v[1]);
			break;

		case 4:		/* 四个轮子各自速度(K230 视觉) */
			for (i = 0; i < MOTOR_NUM; i++) Car_SetWheelSpeed(i, p.v[i]);
			break;

		case 5:		/* 单轮目标速度 */
			Car_SetWheelSpeed(p.id, p.v[0]);
			break;

		case 6:		/* 单轮原始PWM(电机自检用, 不经过 CAR_OPENLOOP_SCALE) */
			if (p.id < MOTOR_NUM)
			{
				Motor_SetSpeed(p.id, p.v[0]);
				s_wheel[p.id].target = (int16_t)(p.v[0] / CAR_OPENLOOP_SCALE);
				s_running = 1;
			}
			break;

		case 8:		/* 四轮同PWM(自检阶段1): 必须一条命令带4个轮子,
					 * 否则循环投递4次会被邮箱合并成"只有最后一个轮子" */
			for (i = 0; i < MOTOR_NUM; i++)
			{
				Motor_SetSpeed(i, p.v[0]);
				s_wheel[i].target = (int16_t)(p.v[0] / CAR_OPENLOOP_SCALE);
			}
			s_running = (p.v[0] != 0) ? 1 : 0;
			break;

		case 7:		/* 四轮制动 */
			for (i = 0; i < MOTOR_NUM; i++) s_wheel[i].target = 0;
			s_running = 0;
			Motor_StopAll();
			break;

		default:
			break;
	}
}
