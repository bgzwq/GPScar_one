/***
	*************************************************************************************************
	*	@file  	lanyacon.c
	*	@brief  ����ҡ��ң�� (4�ֽ�֡: 0xA5 + X + Y + ����)
   *************************************************************************************************
   *  @description
	*
	*  �������ģ�������ϣ���������������ģʽ
	*  �������յ���ָ�����С�����˶�
	*  ���ݸ�ʽ���£�
	*  �ܹ�4�ֽڣ��ɼ�1��ҡ�ˣ�2·ADC��+ 1��������
	*  ֡ͷ��0xA5
	*  x�ᣨ0~255��: 0x00��ʾ���ޣ�0xFF��ʾ�Ҽ���
	*  y�ᣨ0~255��: 0x00��ʾ�ϼ��ޣ�0xFF��ʾ�¼���
	*  ������0x00��ʾδ���£�0x01��ʾ���£����¼���ͣ,
	*  ����ȷ�ϵİ����ط�ת����LED2/LED3����ָͣʾ(�ٰ�һ��Ϩ��)
	*
	*  ʵ��˵��:
	*    1. LanyaCon_Feed() �� main.c �������ֽڴ���(App_BTProcess)���ֽ�ι��,
	*       ����ң��֡���ֽڱ�����(����1), �����ֽڼ����ߵ��ַ�ָ��
	*    2. ҡ��ת����: ���� = ǰ����+ת����/2, ���� = ǰ����-ת����/2
	*    3. �˶����ƽ����ֶ�ģʽ(MODE_MANUAL)����Ч, ��ͣ�κ�ģʽ����Ч
	*    4. ����������������, ԭʼ״̬���ȶ� LANYA_BTN_DEBOUNCE_MS ��ȷ��;
	*       ��ͣͣ����������ִ�в�������(��֤��ȫ), ����ֻ����LED��ת
	*    5. 500ms�ղ�����֡�Զ�ͣ��(���߱���)
	*
	*************************************************************************************************
***/

#include "lanyacon.h"
#include "car.h"
#include "car_led.h"
#include "menu.h"
#include "timer.h"

/*---------------------- ��̬״̬ ----------------------------*/
static LanyaCon_DataTypeDef s_ly;			// ����ң������
static uint8_t  s_rxBuf[3];					// ֡ͷ֮��� X/Y/����
static uint8_t  s_rxCnt  = 0;				// ���յ���֡���ֽ���(0=�ȴ�֡ͷ)
static uint32_t s_lastFrameMs = 0;			// ���һ������֡��ʱ���
static uint8_t  s_wasControlling = 0;		// ��һ���Ƿ��������ҡ����������
static uint8_t  s_btnCandidate = 0;			// ��������: ��ȷ�ϵ�ԭʼ״̬
static uint8_t  s_btnDebounced = 0;			// ��������: ��ȷ�ϵ�״̬
static uint32_t s_btnChangeMs  = 0;			// ��������: ԭʼ״̬���һ�α仯��ʱ���

/*************************************************************************************************
*	�� �� ��:	LanyaCon_DebounceButton
*
*	��������:	��ͣ���������������������ð����ֽ���0/1������, ԭʼ״̬�ȶ�
*				LANYA_BTN_DEBOUNCE_MS ���ȷ��; ȷ�ϰ��µ������ط�תLED2/LED3��
*				ͣ������������������(�� LanyaCon_Apply), ��֤��ͣ������Ч��
*
*************************************************************************************************/
/* PF13/PF9 电平脉冲状态(非阻塞, 由 LanyaCon_Task 到点恢复) */
static uint8_t  s_pf13Pulse = 0;			// 1=PF13高电平脉冲进行中
static uint32_t s_pf13StartMs = 0;
static uint8_t  s_pf9Pulse  = 0;			// 1=PF9低电平脉冲进行中
static uint32_t s_pf9StartMs  = 0;
static uint8_t  s_pf7Pulse  = 0;			// 1=PF7高电平脉冲进行中
static uint32_t s_pf7StartMs  = 0;

/* 轮速斜坡状态: Apply只更新目标值, Task按 LANYA_RAMP_MS/STEP 逐步逼近, 避免轮速突变打滑 */
static int16_t  s_tgtL = 0, s_tgtR = 0;		// 摇杆解算出的目标轮速(左/右)
static int16_t  s_curL = 0, s_curR = 0;		// 当前实际输出轮速(左/右)
static uint32_t s_rampMs = 0;				// 上次斜坡步进时间戳

/*************************************************************************************************
*	函 数 名:	LanyaCon_Init
*
*	函数功能:	功能键输出口初始化: PF13/PF7(默认低电平) / PF9(默认高电平), 推挽输出
*
*************************************************************************************************/
void LanyaCon_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(LANYA_PULSE_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = LANYA_PF13_PIN | LANYA_PF9_PIN | LANYA_PF7_PIN;
	GPIO_Init(LANYA_PULSE_PORT, &GPIO_InitStructure);

	GPIO_ResetBits(LANYA_PULSE_PORT, LANYA_PF13_PIN);	// PF13 默认低电平
	GPIO_ResetBits(LANYA_PULSE_PORT, LANYA_PF7_PIN);	// PF7  默认低电平
	GPIO_SetBits(LANYA_PULSE_PORT, LANYA_PF9_PIN);		// PF9  默认高电平
}

static void LanyaCon_DebounceButton(void)
{
	uint32_t now = Get_Ms();

	if (s_ly.button != s_btnCandidate)
	{
		s_btnCandidate = s_ly.button;	// ԭʼ״̬�仯, ���¼�ʱ
		s_btnChangeMs  = now;
	}
	else if (s_ly.button != s_btnDebounced &&
			 (uint32_t)(now - s_btnChangeMs) >= LANYA_BTN_DEBOUNCE_MS)
	{
		s_btnDebounced = s_ly.button;	// �ȶ���ʱ, ȷ����״̬
		if (s_btnDebounced == LANYA_BTN_STOP)				// ȷ�ϰ��µ�������: ��ת��ָͣʾLED2/LED3
		{
			CarLED_Toggle(CAR_LED2);
			CarLED_Toggle(CAR_LED3);
		}
	}
}

/*************************************************************************************************
*	�� �� ��:	LanyaCon_Apply
*
*	��������:	��һ֡ң������ת��ΪС���˶�(����), ��ͣ����
*
*************************************************************************************************/
static void LanyaCon_Apply(void)
{
	int16_t fwd, turn, left, right;

	LanyaCon_DebounceButton();

	if (s_ly.button == LANYA_BTN_STOP)	// ����=��ͣ, �κ�ģʽ����Ч(����ͣ��, ��������)
	{
		s_tgtL = s_tgtR = 0;
		s_curL = s_curR = 0;	// 急停立即制动, 不走斜坡
		Car_Stop();
		s_wasControlling = 0;
		return;
	}

	if (g_mode != MODE_MANUAL)
	{
		s_wasControlling = 0;
		return;			// ֻ���ֶ�ģʽ��Ӧҡ��
	}

	/* ҡ��ƫ��: ����=ǰ��Ϊ��, ����=��תΪ�� */
	fwd  = (int16_t)LANYA_CENTER - (int16_t)s_ly.y;
	turn = (int16_t)s_ly.x - (int16_t)LANYA_CENTER;

	/* ����: ҡ���ɿ�ʱ�Զ����и���, ��Ϊͣ�� */
	if (fwd  > -LANYA_DEADZONE && fwd  < LANYA_DEADZONE) fwd  = 0;
	if (turn > -LANYA_DEADZONE && turn < LANYA_DEADZONE) turn = 0;

	/* ���ٺϳ� + ӳ�䵽Ŀ���ٶ� */
	/* 差速合成: 横移量按 LANYA_TURN_NUM/DEN 增益叠加为轮速差。
	 * 斜上方 = 边前进边打弯(内外轮差速); 纯左/纯右(fwd=0) = 原地转弯;
	 * 轮速突变由 LanyaCon_Task 的斜坡平滑, 不会打滑 */
	left  = (int16_t)((fwd + turn * LANYA_TURN_NUM / LANYA_TURN_DEN) * LANYA_SPEED_MAX / LANYA_CENTER);
	right = (int16_t)((fwd - turn * LANYA_TURN_NUM / LANYA_TURN_DEN) * LANYA_SPEED_MAX / LANYA_CENTER);

	/* 限幅: 斜角满推时合成值会超过满档, 截到满档 */
	if (left  >  LANYA_SPEED_MAX) left  =  LANYA_SPEED_MAX;
	if (left  < -LANYA_SPEED_MAX) left  = -LANYA_SPEED_MAX;
	if (right >  LANYA_SPEED_MAX) right =  LANYA_SPEED_MAX;
	if (right < -LANYA_SPEED_MAX) right = -LANYA_SPEED_MAX;

	/* 只更新目标值, 实际输出由 LanyaCon_Task 按斜坡逼近;
	 * 摇杆回中(目标0)时也保持"控制中", 让斜坡滑到停车后再交出控制权 */
	s_tgtL = left;
	s_tgtR = right;
	s_wasControlling = 1;
}

/*************************************************************************************************
*	�� �� ��:	LanyaCon_Feed
*
*	��ڲ���:	byte - ���������յ���1�ֽ�
*
*	�� �� ֵ:	1 - ���ֽ�����ң��֡(������), 0 - ��ң��֡�޹�
*
*	��������:	4�ֽ�֡��֡״̬��, �յ�����֡����ִ���˶�����
*
*************************************************************************************************/
uint8_t LanyaCon_Feed(uint8_t byte)
{
	if (s_rxCnt == 0)
	{
		if (byte != LANYA_FRAME_HEAD) return 0;	// ��֡ͷ: ����ң��֡
		s_rxCnt = 1;
		return 1;
	}

	s_rxBuf[s_rxCnt - 1] = byte;
	if (++s_rxCnt < 4) return 1;				// ֡δ����
	s_rxCnt = 0;								// ����4�ֽ�, ��λ����һ֡

	s_ly.x      = s_rxBuf[0];
	s_ly.y      = s_rxBuf[1];

	/* 功能键边沿触发(不用消抖): 0x02 -> PF13高电平300ms, 0x03 -> PF9低电平300ms,
	 * 0x04 -> PF7高电平300ms;
	 * 按住不放也只触发一次, 松手再按才重新触发; 电平恢复由 LanyaCon_Task 计时完成 */
	if (s_rxBuf[2] == LANYA_BTN_PF13_HIGH && s_ly.button != LANYA_BTN_PF13_HIGH)
	{
		GPIO_SetBits(LANYA_PULSE_PORT, LANYA_PF13_PIN);
		s_pf13Pulse   = 1;
		s_pf13StartMs = Get_Ms();
	}
	else if (s_rxBuf[2] == LANYA_BTN_PF9_LOW && s_ly.button != LANYA_BTN_PF9_LOW)
	{
		GPIO_ResetBits(LANYA_PULSE_PORT, LANYA_PF9_PIN);
		s_pf9Pulse   = 1;
		s_pf9StartMs = Get_Ms();
	}
	else if (s_rxBuf[2] == LANYA_BTN_PF7_HIGH && s_ly.button != LANYA_BTN_PF7_HIGH)
	{
		GPIO_SetBits(LANYA_PULSE_PORT, LANYA_PF7_PIN);
		s_pf7Pulse   = 1;
		s_pf7StartMs = Get_Ms();
	}

	s_ly.button = s_rxBuf[2];	// 按键数据位原始值(LANYA_BTN_*)
	s_ly.online = 1;
	s_lastFrameMs = Get_Ms();

	LanyaCon_Apply();
	return 1;
}

/*************************************************************************************************
*	�� �� ��:	LanyaCon_GetData
*
*	��������:	��ȡ����ң������ָ��(�����ڲ˵���ʾ)
*
*************************************************************************************************/
LanyaCon_DataTypeDef *LanyaCon_GetData(void)
{
	return &s_ly;
}

/*************************************************************************************************
*	�� �� ��:	LanyaCon_Task
*
*	��������:	���߱���: ���� LANYA_TIMEOUT_MS û����֡��ͣ��
*
*	��ڲ���:	now - ϵͳ�����(Get_Ms())
*
*	˵    ��:	ֻ��"֮ǰȷʵ����ҡ������������"ʱ��ɲ����
*				ԭ����ʵ��ֻҪ�յ���һ֡, ֮�������е����ϻ���K230�Ӿ�ģʽ,
*				500ms�󶼻������� Car_Stop(), ������ģʽ�����̿���Ȩ��
*
*************************************************************************************************/
/* 斜坡步进: cur 向 tgt 靠近一步 */
static int16_t LanyaCon_RampStep(int16_t cur, int16_t tgt)
{
	if (cur < tgt) { cur += LANYA_RAMP_STEP; if (cur > tgt) cur = tgt; }
	else if (cur > tgt) { cur -= LANYA_RAMP_STEP; if (cur < tgt) cur = tgt; }
	return cur;
}

void LanyaCon_Task(uint32_t now)
{
	/* PF13/PF9/PF7电平脉冲到点恢复(非阻塞, 不影响其它任务) */
	if (s_pf13Pulse && (uint32_t)(now - s_pf13StartMs) >= LANYA_PULSE_MS)
	{
		s_pf13Pulse = 0;
		GPIO_ResetBits(LANYA_PULSE_PORT, LANYA_PF13_PIN);	// PF13恢复低电平
	}
	if (s_pf9Pulse && (uint32_t)(now - s_pf9StartMs) >= LANYA_PULSE_MS)
	{
		s_pf9Pulse = 0;
		GPIO_SetBits(LANYA_PULSE_PORT, LANYA_PF9_PIN);		// PF9恢复高电平
	}
	if (s_pf7Pulse && (uint32_t)(now - s_pf7StartMs) >= LANYA_PULSE_MS)
	{
		s_pf7Pulse = 0;
		GPIO_ResetBits(LANYA_PULSE_PORT, LANYA_PF7_PIN);	// PF7恢复低电平
	}

	/* 轮速斜坡: 目标值由 Apply 更新, 这里每 LANYA_RAMP_MS 逼近一步, 轮速不突变 */
	if (s_wasControlling && g_mode == MODE_MANUAL)
	{
		if ((uint32_t)(now - s_rampMs) >= LANYA_RAMP_MS)
		{
			int16_t newL, newR;

			s_rampMs = now;
			newL = LanyaCon_RampStep(s_curL, s_tgtL);
			newR = LanyaCon_RampStep(s_curR, s_tgtR);

			if (newL != s_curL || newR != s_curR)	// 有变化才写电机
			{
				s_curL = newL;
				s_curR = newR;
				Car_SetWheelSpeed(MOTOR_1, s_curL);
				Car_SetWheelSpeed(MOTOR_2, s_curL);
				Car_SetWheelSpeed(MOTOR_3, s_curR);
				Car_SetWheelSpeed(MOTOR_4, s_curR);
			}

			/* 摇杆回中后斜坡滑到0 -> 真正制动, 并把底盘控制权交还给其它控制源 */
			if (s_tgtL == 0 && s_tgtR == 0 && s_curL == 0 && s_curR == 0)
			{
				s_wasControlling = 0;
				Car_Stop();
			}
		}
	}
	else
	{
		s_curL = 0;
		s_curR = 0;		// 没在控制时清零, 下次接管从0开始爬升
	}

	if (s_ly.online && (uint32_t)(now - s_lastFrameMs) > LANYA_TIMEOUT_MS)
	{
		s_ly.online = 0;
		if (s_wasControlling)
		{
			s_wasControlling = 0;
			Car_Stop();
		}
	}
}
