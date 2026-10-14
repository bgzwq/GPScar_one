/***
	*************************************************************************************************
	*	@file  	navlog.c
	*	@brief  行程记录 / SD卡存储(FatFs): GPS轨迹点 + TRACK.CSV + MAP.BMP
   *************************************************************************************************
   *  @description
	*
	*  1. NavLog_Init(): f_mount 挂载SD卡 -> f_mkdir 建 CARLOG 目录 ->
	*     f_open 追加方式打开 TRACK.CSV 并写入表头
	*  2. NavLog_Update(): 每100ms调用一次。只在"定位更新序号fixSeq变化"时处理,
	*     经过 精度过滤 后把点存进RAM数组, 同时追加一行CSV
	*  3. NavLog_WriteBmp1bpp(): 地图页面按"保存"时, 把 mapview 生成的
	*     1bpp 位图逐行转成 24位BMP写到SD卡
	*
	*  内存说明: 全部静态分配, 不使用malloc。
	*    - 轨迹数组 1024 × 8B = 8KB
	*    - BMP行缓冲 240 × 3B = 720B (地图区域宽236, 静态缓冲按上限240开)
	*    FatFs 的 FATFS/FIL 已经不再需要4KB扇区缓冲(ffconf.h 已把 FF_MAX_SS 固定512)
	*
	*************************************************************************************************
***/

#include "navlog.h"
#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "location.h"
#include "gps.h"
#include "timer.h"

/*---------------------- 静态数据 ----------------------------*/
static FATFS s_fs;								// 文件系统对象
static FIL   s_csv;								// TRACK.CSV 文件对象
static uint8_t s_sdMounted = 0;					// 1=已挂载
static uint8_t s_sdOk      = 0;					// 1=目录/文件都准备好了
static uint8_t s_csvOpen   = 0;					// 1=TRACK.CSV 文件对象处于打开状态

static NavPoint_TypeDef s_pts[NAVLOG_MAX_POINTS];	// 轨迹点
static uint32_t s_count    = 0;						// 已记录点数
static uint32_t s_lastSeq  = 0;						// 上次处理的定位序号
static uint32_t s_lastPtMs = 0;						// 上一个记录点的时间戳(算隐含速度用)
static uint32_t s_lastRetryMs = 0;					// 上次尝试重新打开CSV的时间戳
static uint32_t s_reject   = 0;						// 被过滤掉的次数
static uint8_t  s_rejectRun = 0;					// 连续被过滤次数
static uint32_t s_sinceSync = 0;					// 距上次f_sync写入的点数
static float    s_totalM   = 0;						// 轨迹累计长度(m)
static uint32_t s_csvRows  = 0;						// CSV累计行号(跨上电续写, 这里只统计本次)

static uint8_t  s_bmpRow[NAVLOG_BMP_MAX_W * 3];		// BMP行缓冲(静态, 避免占用栈)

/*---------------------- 小工具 ----------------------------*/

static uint8_t NavLog_OpenCsv(void)
{
	FRESULT fr;
	UINT    bw;

	/* 之前写失败过的话文件对象可能还开着, 先关掉再重新打开 */
	if (s_csvOpen)
	{
		f_close(&s_csv);
		s_csvOpen = 0;
	}

	fr = f_open(&s_csv, NAVLOG_CSV_PATH, FA_WRITE | FA_OPEN_APPEND);
	if (fr != FR_OK)
	{
		return 0;
	}
	s_csvOpen = 1;

	if (f_size(&s_csv) == 0)
	{
		static const char hdr[] =
			"idx,lat,lon,speed_kmh,course_deg,sats,hdop,fix,source,utc,t_ms\r\n";
		f_write(&s_csv, hdr, (UINT)(sizeof(hdr) - 1), &bw);
		f_sync(&s_csv);
	}

	s_sdOk      = 1;
	s_sinceSync = 0;
	s_csvRows   = 0;	// idx 是"本次上电"内的序号, 文件是跨上电续写的
	return 1;
}

static void NavLog_PutU16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void NavLog_PutU32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
	p[2] = (uint8_t)((v >> 16) & 0xFF);
	p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* E7定点 -> 十进制度(仅用于打印到CSV, 不参与运算) */
static void NavLog_E7ToText(int32_t e7, char *out, uint8_t outSize)
{
	int32_t ip = e7 / 10000000;
	int32_t fp = e7 % 10000000;

	if (fp < 0) fp = -fp;
	snprintf(out, outSize, "%ld.%07ld", (long)ip, (long)fp);
}

/*---------------------- 初始化 ----------------------------*/

/*************************************************************************************************
*	函 数 名:	NavLog_Init
*
*	返 回 值:	1 - SD卡可用(已挂载, 目录与CSV文件已就绪); 0 - 无SD卡/挂载失败
*
*	函数功能:	挂载SD卡, 建 CARLOG 目录, 以追加方式打开 TRACK.CSV 并写表头
*
*	说    明:	SD_Init() 首次上卡比较慢(几百ms), 请在开机初始化阶段调用。
*				即使返回0, 轨迹依然会在RAM里记录并能在LCD地图页显示。
*
*************************************************************************************************/
uint8_t NavLog_Init(void)
{
	FRESULT fr;
	DIR     dir;

	/* 1. 挂载 */
	fr = f_mount(&s_fs, "0:", 1);
	if (fr != FR_OK)
	{
		return 0;
	}
	s_sdMounted = 1;

	/* 2. 建立 CARLOG 目录(已存在会返回 FR_EXIST, 属正常) */
	fr = f_opendir(&dir, NAVLOG_DIR);
	if (fr == FR_OK)
	{
		f_closedir(&dir);
	}
	else
	{
		fr = f_mkdir(NAVLOG_DIR);
		if (fr != FR_OK && fr != FR_EXIST)
		{
			return 0;
		}
	}

	/* 3. 追加方式打开CSV并写表头 */
	if (!NavLog_OpenCsv()) return 0;

	return 1;
}

/*---------------------- 轨迹记录 ----------------------------*/

/*************************************************************************************************
*	函 数 名:	NavLog_Update
*
*	入口参数:	now - 系统毫秒戳(Get_Ms())
*
*	函数功能:	有新定位点时, 按精度规则过滤后记录轨迹点并追加CSV
*
*	说    明:	建议100ms调用一次; 内部用 fixSeq 判断是否真的有新定位,
*				所以不会重复记录同一个点。
*
*************************************************************************************************/
void NavLog_Update(uint32_t now)
{
	Location_DataTypeDef *loc = Location_GetData();
	GPS_DataTypeDef      *gps = GPS_GetData();
	NavPoint_TypeDef      p;
	float d = 0.0f;
	char  latTxt[16], lonTxt[16];
	char  utcTxt[24];
	char  line[128];
	int   n;
	UINT  bw;

	/* SD卡挂上了但文件没打开成功(例如上电时卡还没准备好) -> 定期重试,
	 * 这样插卡晚一点也能自动开始记录, 不需要重新上电 */
	if (!s_sdOk && s_sdMounted && (uint32_t)(now - s_lastRetryMs) >= NAVLOG_RETRY_MS)
	{
		s_lastRetryMs = now;
		NavLog_OpenCsv();
	}

	/* 只在真的有新GPS定位时处理 */
	if (loc->fixSeq == s_lastSeq) return;
	s_lastSeq = loc->fixSeq;

	/* --- 精度过滤1: 定位必须有效, 且来源是GPS而不是惯导推算 --- */
	if (!gps->valid || loc->source != LOC_SRC_GPS)
	{
		s_reject++;
		return;
	}

	/* --- 精度过滤2: 卫星数与HDOP(差的质量的点不进地图) --- */
	if (gps->satellites < NAVLOG_MIN_SATS) return;
	if (gps->hdop > NAVLOG_MAX_HDOP)        return;
	if (gps->fixQuality == 0)               return;

	p.latE7 = loc->latE7;
	p.lonE7 = loc->lonE7;

	/* --- 精度过滤3: 最小距离 + 野值(跳点)抑制 --- */
	if (s_count > 0)
	{
		float dt;

		d = Location_E7DistanceM(s_pts[s_count - 1].latE7, s_pts[s_count - 1].lonE7,
		                         p.latE7, p.lonE7);

		if (d < NAVLOG_MIN_DIST_M) return;			// 基本没动, 不记(静止时不会画成一团)

		/* 相邻两点隐含速度过大 -> 判为GPS野值, 丢弃。
		 * 连续丢弃多次后强制接受(见 NAVLOG_REJECT_RESYNC), 避免定位整体漂移时永久锁死。 */
		dt = (float)(now - s_lastPtMs) / 1000.0f;
		if (s_rejectRun < NAVLOG_REJECT_RESYNC && dt > 0.05f &&
			(d / dt) > NAVLOG_MAX_SPEED_MS)
		{
			s_reject++;
			s_rejectRun++;
			return;
		}
		s_rejectRun = 0;
	}

	/* --- 记录到RAM --- */
	if (s_count < NAVLOG_MAX_POINTS)
	{
		s_pts[s_count] = p;
		s_count++;
		if (s_count >= 2) s_totalM += d;
	}
	s_lastPtMs = now;

	/* --- 追加CSV(没有SD卡就只留在RAM里) --- */
	if (!s_sdOk) return;

	NavLog_E7ToText(p.latE7, latTxt, sizeof(latTxt));
	NavLog_E7ToText(p.lonE7, lonTxt, sizeof(lonTxt));

	/* GPS的UTC时间: 让日志跨上电也能对上(没有RTC, 只能用GPS时间) */
	snprintf(utcTxt, sizeof(utcTxt), "%04u-%02u-%02uT%02u:%02u:%02u",
	         (unsigned)gps->year, (unsigned)gps->month, (unsigned)gps->day,
	         (unsigned)gps->hour, (unsigned)gps->minute, (unsigned)gps->second);

	n = snprintf(line, sizeof(line),
	             "%lu,%s,%s,%.1f,%.1f,%u,%.1f,%u,%u,%s,%lu\r\n",
	             (unsigned long)s_csvRows,
	             latTxt, lonTxt,
	             loc->speedKmh, loc->course,
	             (unsigned)gps->satellites,
	             gps->hdop,
	             (unsigned)gps->fixQuality,
	             (unsigned)loc->source,
	             utcTxt,
	             (unsigned long)now);
	if (n > 0)
	{
		if (f_write(&s_csv, line, (UINT)n, &bw) != FR_OK || bw != (UINT)n)
		{
			f_close(&s_csv);		// 释放文件对象, 之后由重试逻辑重新打开
			s_csvOpen = 0;
			s_sdOk    = 0;
			return;
		}
		s_csvRows++;

		/* 定期落盘: 既限制掉电丢失量, 也避免频繁写卡 */
		if (++s_sinceSync >= NAVLOG_SYNC_EVERY)
		{
			s_sinceSync = 0;
			f_sync(&s_csv);
		}
	}
}

/*---------------------- 查询接口 ----------------------------*/

uint32_t NavLog_GetCount(void)
{
	return s_count;
}

const NavPoint_TypeDef *NavLog_GetPoints(void)
{
	return s_pts;
}

NavLogSdState_TypeDef NavLog_GetStateRaw(void)
{
	if (!s_sdOk) return NAVLOG_SD_NONE;
	if (s_count >= NAVLOG_MAX_POINTS) return NAVLOG_SD_FULL;
	return NAVLOG_SD_OK;
}

NavLogSdState_TypeDef NavLog_GetState(void)
{
	return NavLog_GetStateRaw();
}

const char *NavLog_GetStateName(void)
{
	static const char *s_name[3] = { "NONE", "OK  ", "FULL" };
	return s_name[NavLog_GetStateRaw()];
}

uint8_t NavLog_IsSdOk(void)
{
	return s_sdOk;
}

uint32_t NavLog_GetRejectCount(void)
{
	return s_reject;
}

float NavLog_GetDistanceM(void)
{
	return s_totalM;
}

void NavLog_Clear(void)
{
	s_count     = 0;
	s_totalM    = 0.0f;
	s_rejectRun = 0;
	s_lastPtMs  = 0;
}

void NavLog_Sync(void)
{
	if (s_sdOk) f_sync(&s_csv);
}

/*---------------------- BMP导出 ----------------------------*/

/*************************************************************************************************
*	函 数 名:	NavLog_WriteBmp1bpp
*
*	入口参数:	bits - 1bpp位图数据(每行按字节对齐, MSB在左, 1=前景)
*				w,h  - 宽高(<=NAVLOG_BMP_MAX_W)
*				fg   - 前景色 24位RGB888
*				bg   - 背景色 24位RGB888
*
*	返 回 值:	1 - 成功; 0 - 失败(无SD卡/写错误)
*
*	函数功能:	把1bpp位图写成 24位BMP 文件(0:/CARLOG/MAP.BMP)
*
*	说    明:	1. 逐行写, 只需要一行(708B)的静态缓冲, 不占大内存
*				2. BMP像素是BGR顺序且自下而上存放, 这里已处理好
*				3. 尺寸是地图区域大小(236x192), 文件约136KB, 保存瞬间界面会停顿
*				   一两百毫秒(用户主动按键触发, 可接受)
*
*************************************************************************************************/
uint8_t NavLog_WriteBmp1bpp(const uint8_t *bits, uint16_t w, uint16_t h,
                            uint32_t fg, uint32_t bg)
{
	FIL     f;
	FRESULT fr;
	UINT    bw;
	uint8_t hdr[54];
	uint16_t rowBytes;
	uint32_t rowSize;
	uint32_t imageSize;
	uint32_t fileSize;
	uint16_t y, x;
	uint8_t  fgB = (uint8_t)(fg & 0xFF), fgG = (uint8_t)((fg >> 8) & 0xFF), fgR = (uint8_t)((fg >> 16) & 0xFF);
	uint8_t  bgB = (uint8_t)(bg & 0xFF), bgG = (uint8_t)((bg >> 8) & 0xFF), bgR = (uint8_t)((bg >> 16) & 0xFF);

	if (!s_sdOk)          return 0;
	if (w == 0 || h == 0) return 0;
	if (w > NAVLOG_BMP_MAX_W) return 0;

	rowBytes  = (uint16_t)((w + 7) / 8);
	rowSize   = ((uint32_t)w * 3u + 3u) & ~3u;		// 4字节对齐
	imageSize = rowSize * h;
	fileSize  = 54u + imageSize;

	/* ---- 文件头(14字节) ---- */
	hdr[0] = 'B'; hdr[1] = 'M';
	NavLog_PutU32(&hdr[2],  fileSize);
	NavLog_PutU32(&hdr[6],  0);
	NavLog_PutU32(&hdr[10], 54);

	/* ---- 信息头(40字节) ---- */
	NavLog_PutU32(&hdr[14], 40);
	NavLog_PutU32(&hdr[18], (uint32_t)w);
	NavLog_PutU32(&hdr[22], (uint32_t)h);	// 正值 = 自下而上
	NavLog_PutU16(&hdr[26], 1);
	NavLog_PutU16(&hdr[28], 24);			// 24位色
	NavLog_PutU32(&hdr[30], 0);				// 不压缩
	NavLog_PutU32(&hdr[34], imageSize);
	NavLog_PutU32(&hdr[38], 2835);			// 约72DPI
	NavLog_PutU32(&hdr[42], 2835);
	NavLog_PutU32(&hdr[46], 0);
	NavLog_PutU32(&hdr[50], 0);

	fr = f_open(&f, NAVLOG_BMP_PATH, FA_WRITE | FA_CREATE_ALWAYS);
	if (fr != FR_OK)
	{
		return 0;
	}

	if (f_write(&f, hdr, 54, &bw) != FR_OK || bw != 54)
	{
		f_close(&f);
		return 0;
	}

	/* ---- 像素数据: 自下而上, 每行BGR ---- */
	for (y = 0; y < h; y++)
	{
		const uint8_t *srcRow = bits + (uint32_t)(h - 1 - y) * rowBytes;

		for (x = 0; x < w; x++)
		{
			uint8_t bit = (uint8_t)((srcRow[x >> 3] >> (7 - (x & 7))) & 0x01);

			s_bmpRow[x * 3 + 0] = bit ? fgB : bgB;
			s_bmpRow[x * 3 + 1] = bit ? fgG : bgG;
			s_bmpRow[x * 3 + 2] = bit ? fgR : bgR;
		}
		/* 行末补齐到4字节 */
		for (x = (uint16_t)(w * 3); x < (uint16_t)rowSize; x++) s_bmpRow[x] = 0;

		if (f_write(&f, s_bmpRow, (UINT)rowSize, &bw) != FR_OK || bw != (UINT)rowSize)
		{
			f_close(&f);
			return 0;
		}
	}

	f_sync(&f);
	f_close(&f);

	return 1;
}
