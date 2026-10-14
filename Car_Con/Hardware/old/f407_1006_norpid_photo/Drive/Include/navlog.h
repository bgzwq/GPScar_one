#ifndef __NAVLOG_H
#define __NAVLOG_H

#include "stm32f4xx.h"

/*=================================================================================================
 * 行程记录 / SD卡存储 (FatFs)
 *-------------------------------------------------------------------------------------------------
 * 功能:
 *   1. 挂载SD卡, 建立 CARLOG 目录
 *   2. 把GPS轨迹按"精度过滤"后记录到RAM数组(供LCD画地图) 并实时追加到 TRACK.CSV
 *   3. 支持把一张1bpp位图写成 24位 BMP 存到SD卡(由 mapview.c 生成地图位图)
 *
 * 精度过滤(直击"GPS定位精度"):
 *   - 定位无效 / 卫星数不足 / HDOP过大 时不记录(避免把差分解算进地图)
 *   - 与上一点距离小于 NAVLOG_MIN_DIST_M 不记录(静止时GPS抖动不会画成一团)
 *   - 相邻两点隐含速度超过 NAVLOG_MAX_SPEED_MS 判为野值丢弃(跳点抑制)
 *   - 连续丢弃超过 NAVLOG_REJECT_RESYNC 次则强制接受新点, 防止永久锁死
 *
 * 存储位置: 0:/CARLOG/TRACK.CSV  (追加, 每次上电续写; idx列是本次上电内的序号,
 *                                  utc列是GPS时间, 跨上电也能对上)
 *           0:/CARLOG/MAP.BMP    (每次保存覆盖, 236x192 24位, 1bpp黑白风格轨迹图)
 *===============================================================================================*/

/*---------------------- 容量与路径 ----------------------------*/
#define NAVLOG_MAX_POINTS	1024	// RAM轨迹点上限(1.0m间隔约可记录1km)
#define NAVLOG_DIR			"0:/CARLOG"
#define NAVLOG_CSV_PATH		"0:/CARLOG/TRACK.CSV"
#define NAVLOG_BMP_PATH		"0:/CARLOG/MAP.BMP"

/*---------------------- 精度过滤参数 ----------------------------*/
#define NAVLOG_MIN_DIST_M		1.0f	// 与上一点最小距离(m), 小于该值不记录
#define NAVLOG_MIN_SATS			4		// 最少卫星数
#define NAVLOG_MAX_HDOP			3.0f	// 最大HDOP(水平精度因子)
#define NAVLOG_MAX_SPEED_MS		8.0f	// 相邻点隐含最大速度(m/s), 约29km/h
#define NAVLOG_REJECT_RESYNC	5		// 连续丢弃多少次后强制接受新点
#define NAVLOG_SYNC_EVERY		10		// 每记录多少个点 f_sync 一次
#define NAVLOG_RETRY_MS			5000	// 挂载成功但文件没打开时的重试间隔(ms)

/*---------------------- 轨迹点 ----------------------------*/
typedef struct
{
	int32_t latE7;		// 纬度 × 1e7
	int32_t lonE7;		// 经度 × 1e7
} NavPoint_TypeDef;

/*---------------------- SD卡状态 ----------------------------*/
typedef enum
{
	NAVLOG_SD_NONE = 0,	// 无SD卡/挂载失败(轨迹仍会在RAM里记录并显示)
	NAVLOG_SD_OK,		// 已挂载, 正常写入
	NAVLOG_SD_FULL,		// RAM轨迹点已满(CSV仍在追加)
} NavLogSdState_TypeDef;

/*---------------------- 接口 ----------------------------*/

/*
 * 初始化: 挂载SD卡, 建立 CARLOG 目录并打开 TRACK.CSV(追加)。
 * 返回 1=SD可用, 0=没有SD卡(不影响LCD地图显示, 只是不落盘)。
 * 说明: SD_Init() 首次上卡可能需要几百毫秒, 请在开机初始化阶段调用。
 */
uint8_t NavLog_Init(void);

/* 周期性调用(建议100ms): 有新定位点时按精度规则记录 */
void NavLog_Update(uint32_t now);

uint32_t 		NavLog_GetCount(void);			// 已记录轨迹点数
const NavPoint_TypeDef *NavLog_GetPoints(void);// 轨迹点数组
NavLogSdState_TypeDef NavLog_GetState(void);	// SD状态
const char 	   *NavLog_GetStateName(void);		// 状态名(定宽4字符, 供LCD显示)
NavLogSdState_TypeDef NavLog_GetStateRaw(void);	// 原始状态(未经FULL判定)

uint32_t NavLog_GetRejectCount(void);			// 被精度过滤丢弃的定位点次数
float 	 NavLog_GetDistanceM(void);			// 轨迹累计长度(m)
uint8_t  NavLog_IsSdOk(void);					// SD是否可写

void NavLog_Clear(void);						// 清空RAM绘图轨迹(不影响已写CSV)
void NavLog_Sync(void);							// 把CSV缓存刷到卡上

/*
 * 把1bpp位图写成 24位BMP 到 NAVLOG_BMP_PATH。
 *   bits   - 位图数据, 每行按字节对齐, MSB在左, 1=前景
 *   w,h    - 尺寸(<=NAVLOG_BMP_MAX_W)
 *   fg/bg  - 前景/背景色, 24位RGB888
 * 返回 1=成功。
 */
#define NAVLOG_BMP_MAX_W	240
uint8_t NavLog_WriteBmp1bpp(const uint8_t *bits, uint16_t w, uint16_t h,
                            uint32_t fg, uint32_t bg);

#endif /* __NAVLOG_H */
