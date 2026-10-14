/***
	*************************************************************************************************
	*	@file  	mapview.c
	*	@brief  轨迹地图显示(LCD) + 地图导出(1bpp -> 24位BMP)
   *************************************************************************************************
   *  @description
	*
	*  投影约定: 北朝上(X=东, Y=屏幕向下=南)。
	*  所有运算都在"局部平面米"里做(Location_E7ToLocalM):
	*  先用 int32 的 E7 经纬度做差再乘系数, 不经过 float 经纬度,
	*  所以不会出现 float 在 113°E 附近约 0.8m 的量化台阶。
	*
	*  两张画布共用同一套投影:
	*    1. LCD  : 直接画在 MAP_AREA_* 指定的区域里
	*    2. 位图 : s_bits[] 1bpp 缓冲, 交给 navlog 写成 BMP
	*
	*************************************************************************************************
***/

#include "mapview.h"
#include <math.h>
#include <stdlib.h>
#include "lcd_spi_154.h"
#include "navlog.h"
#include "location.h"

/*---------------------- 位图缓冲 ----------------------------*/
#define MAP_BMP_ROWBYTES	(((MAP_AREA_W) + 7) / 8)
static uint8_t s_bits[MAP_BMP_ROWBYTES * MAP_AREA_H];

/*
 * 画线用的颜色。
 * 说明: LCD_DrawPoint() 现在会使用传入的 color 形参(见 lcd_spi_154.c),
 * 而 lcd_spi_154.c 里的全局 LCD 结构体并没有在头文件里 extern,
 * 所以这里自己保存当前图层的颜色, 不直接访问 LCD.Color。
 */
static uint32_t s_lineColor = 0xFFFFFF;

/*---------------------- 视图状态 ----------------------------*/
static int32_t s_orgLatE7 = 0;			// 投影原点(轨迹第一个点)
static int32_t s_orgLonE7 = 0;
static float   s_cx = 0.0f;				// 视野中心(相对原点的局部米)
static float   s_cy = 0.0f;
static float   s_baseScale = 1.0f;		// 自动缩放得到的 像素/米
static float   s_scale = 1.0f;			// 实际使用 = s_baseScale * s_zoom
static float   s_zoom = 1.0f;			// 手动缩放倍数
static float   s_gridM = 10.0f;			// 当前网格间距(m)

static uint8_t  s_wasVisible = 0;		// 上一拍地图页是否可见
static uint8_t  s_forceRedraw = 1;		// 强制重绘标志
static uint32_t s_drawnCount = 0xFFFFFFFF;
static int32_t  s_drawnLatE7 = 0;
static int32_t  s_drawnLonE7 = 0;

/* 把1bpp位图画到LCD时用的前景色(与导出BMP保持一致) */
#define MAP_COLOR_BG		0x000000	// 背景: 黑
#define MAP_COLOR_GRID		0x1A3A1A	// 网格: 暗绿
#define MAP_COLOR_TRACK		0x40FF40	// 轨迹: 亮绿
#define MAP_COLOR_START		0x00FFFF	// 起点: 青
#define MAP_COLOR_NOW		0xFF4040	// 当前位置: 红
#define MAP_COLOR_FRAME		0x606060	// 边框: 灰

#define MAP_BMP_FG			0x40FF40	// 导出BMP的前景色
#define MAP_BMP_BG			0x000000	// 导出BMP的背景色

/*---------------------- 内部函数 ----------------------------*/

/* 局部平面米 -> 地图区域内坐标(像素)。不做裁剪, 由调用者判断。 */
static void MapView_ProjectM(float northM, float eastM, int *ax, int *ay)
{
	*ax = (int)((float)MAP_AREA_W * 0.5f + (eastM  - s_cx) * s_scale + 0.5f);
	*ay = (int)((float)MAP_AREA_H * 0.5f - (northM - s_cy) * s_scale + 0.5f);
}

static void MapView_Project(int32_t latE7, int32_t lonE7, int *ax, int *ay)
{
	float n = 0.0f, e = 0.0f;

	Location_E7ToLocalM(s_orgLatE7, s_orgLonE7, latE7, lonE7, &n, &e);
	MapView_ProjectM(n, e, ax, ay);
}

static uint8_t MapView_InArea(int ax, int ay)
{
	return (ax >= 0 && ax < MAP_AREA_W && ay >= 0 && ay < MAP_AREA_H) ? 1 : 0;
}

/*---------------------- 网格间距 ----------------------------*/

static float MapView_NiceGridM(void)
{
	static const float nice[] = { 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f,
	                              50.0f, 100.0f, 200.0f, 500.0f, 1000.0f, 2000.0f };
	uint8_t i;

	for (i = 0; i < sizeof(nice) / sizeof(nice[0]); i++)
	{
		if (nice[i] * s_scale >= 32.0f) return nice[i];
	}
	return nice[sizeof(nice) / sizeof(nice[0]) - 1];
}

/*---------------------- 自动缩放 ----------------------------*/

static void MapView_RecalcFit(void)
{
	const NavPoint_TypeDef *pts = NavLog_GetPoints();
	uint32_t cnt = NavLog_GetCount();
	uint32_t i;
	float n, e;
	float nMin = 0, nMax = 0, eMin = 0, eMax = 0;
	float spanN, spanE;
	float lastN = 0, lastE = 0;

	/* 原点取第一个轨迹点; 没有点时用当前位置 */
	if (cnt > 0)
	{
		s_orgLatE7 = pts[0].latE7;
		s_orgLonE7 = pts[0].lonE7;
	}
	else
	{
		Location_DataTypeDef *loc = Location_GetData();
		s_orgLatE7 = loc->latE7;
		s_orgLonE7 = loc->lonE7;
		s_cx = 0.0f;
		s_cy = 0.0f;
		s_baseScale = (float)MAP_AREA_W / MAP_MIN_SPAN_M;
		s_scale = s_baseScale * s_zoom;
		s_gridM = MapView_NiceGridM();
		return;
	}

	for (i = 0; i < cnt; i++)
	{
		Location_E7ToLocalM(s_orgLatE7, s_orgLonE7, pts[i].latE7, pts[i].lonE7, &n, &e);

		if (i == 0) { nMin = nMax = n; eMin = eMax = e; }
		else
		{
			if (n < nMin) nMin = n;
			if (n > nMax) nMax = n;
			if (e < eMin) eMin = e;
			if (e > eMax) eMax = e;
		}
		lastN = n;
		lastE = e;
	}

	spanN = nMax - nMin;
	spanE = eMax - eMin;
	if (spanN < MAP_MIN_SPAN_M) spanN = MAP_MIN_SPAN_M;
	if (spanE < MAP_MIN_SPAN_M) spanE = MAP_MIN_SPAN_M;

	/* 留15%边距 */
	spanN *= 1.15f;
	spanE *= 1.15f;

	/* 取能同时容纳横纵跨度的缩放(像素/米) */
	{
		float sN = (float)MAP_AREA_H / spanN;
		float sE = (float)MAP_AREA_W / spanE;
		s_baseScale = (sN < sE) ? sN : sE;
	}
	s_scale = s_baseScale * s_zoom;

	/* 放大查看时以当前位置为中心, 否则以整条轨迹的包围盒中心为中心 */
	if (s_zoom > 1.001f || cnt == 1)
	{
		s_cx = lastE;
		s_cy = lastN;
	}
	else
	{
		s_cx = (eMin + eMax) * 0.5f;
		s_cy = (nMin + nMax) * 0.5f;
	}

	s_gridM = MapView_NiceGridM();
}

/*---------------------- 画线(两个后端) ----------------------------*/

static void MapView_LineLcd(int x0, int y0, int x1, int y1)
{
	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = dx + dy;

	for (;;)
	{
		if (MapView_InArea(x0, y0))
		{
			LCD_DrawPoint((uint16_t)(MAP_AREA_X + x0), (uint16_t)(MAP_AREA_Y + y0), s_lineColor);
		}
		if (x0 == x1 && y0 == y1) break;

		{
			int e2 = 2 * err;
			if (e2 >= dy) { err += dy; x0 += sx; }
			if (e2 <= dx) { err += dx; y0 += sy; }
		}
	}
}

static void MapView_BmpPixel(int x, int y, uint8_t on)
{
	uint8_t *p;

	if (!MapView_InArea(x, y)) return;

	p = &s_bits[(uint32_t)y * MAP_BMP_ROWBYTES + (x >> 3)];
	if (on) *p |=  (uint8_t)(0x80 >> (x & 7));
	else    *p &= (uint8_t)~(0x80 >> (x & 7));
}

static void MapView_LineBmp(int x0, int y0, int x1, int y1, uint8_t on)
{
	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = dx + dy;

	for (;;)
	{
		MapView_BmpPixel(x0, y0, on);
		if (x0 == x1 && y0 == y1) break;

		{
			int e2 = 2 * err;
			if (e2 >= dy) { err += dy; x0 += sx; }
			if (e2 <= dx) { err += dx; y0 += sy; }
		}
	}
}

static void MapView_FillBmp(int x, int y, int r)
{
	int i, j;

	for (j = -r; j <= r; j++)
		for (i = -r; i <= r; i++)
			if (i * i + j * j <= r * r) MapView_BmpPixel(x + i, y + j, 1);
}

/* 网格: 以原点为中心按 s_gridM 的整数倍画线(两个后端共用同一套循环) */
static void MapView_DrawGrid(uint8_t toBmp)
{
	int i;
	int cxPix = (int)((float)MAP_AREA_W * 0.5f - s_cx * s_scale + 0.5f);	// 原点(0,0)在区域里的位置
	int cyPix = (int)((float)MAP_AREA_H * 0.5f + s_cy * s_scale + 0.5f);
	int step  = (int)(s_gridM * s_scale + 0.5f);

	if (step < 4) return;

	/* 垂直线 */
	for (i = cxPix % step; i < MAP_AREA_W; i += step)
	{
		if (i < 0) continue;
		if (toBmp) MapView_LineBmp(i, 0, i, MAP_AREA_H - 1, 1);
		else       MapView_LineLcd(i, 0, i, MAP_AREA_H - 1);
	}
	/* 水平线 */
	for (i = cyPix % step; i < MAP_AREA_H; i += step)
	{
		if (i < 0) continue;
		if (toBmp) MapView_LineBmp(0, i, MAP_AREA_W - 1, i, 1);
		else       MapView_LineLcd(0, i, MAP_AREA_W - 1, i);
	}
}

/* 指北箭头(固定在区域右上角) */
static void MapView_DrawNorth(uint8_t toBmp)
{
	int x = MAP_AREA_W - 16;
	int y = 24;

	if (toBmp)
	{
		MapView_LineBmp(x, y, x, y - 14, 1);
		MapView_LineBmp(x, y - 14, x - 4, y - 8, 1);
		MapView_LineBmp(x, y - 14, x + 4, y - 8, 1);
	}
	else
	{
		MapView_LineLcd(x, y, x, y - 14);
		MapView_LineLcd(x, y - 14, x - 4, y - 8);
		MapView_LineLcd(x, y - 14, x + 4, y - 8);
	}
}

/* 比例尺(左下角, 长度=1个网格) */
static void MapView_DrawScaleBar(uint8_t toBmp)
{
	int len = (int)(s_gridM * s_scale + 0.5f);
	int x0 = 8;
	int y0 = MAP_AREA_H - 10;
	int x1 = x0 + len;

	if (len < 4) return;
	if (x1 > MAP_AREA_W - 8) x1 = MAP_AREA_W - 8;

	if (toBmp)
	{
		MapView_LineBmp(x0, y0, x1, y0, 1);
		MapView_LineBmp(x0, y0 - 3, x0, y0 + 3, 1);
		MapView_LineBmp(x1, y0 - 3, x1, y0 + 3, 1);
	}
	else
	{
		MapView_LineLcd(x0, y0, x1, y0);
		MapView_LineLcd(x0, y0 - 3, x0, y0 + 3);
		MapView_LineLcd(x1, y0 - 3, x1, y0 + 3);
	}
}

/*---------------------- LCD绘制 ----------------------------*/

static void MapView_DrawLcd(void)
{
	const NavPoint_TypeDef *pts = NavLog_GetPoints();
	uint32_t cnt = NavLog_GetCount();
	uint32_t i;
	int ax, ay, px = 0, py = 0;
	uint8_t havePrev = 0;

	/* 背景 + 边框 */
	LCD_SetBackColor(MAP_COLOR_BG);
	LCD_ClearRect(MAP_AREA_X, MAP_AREA_Y, MAP_AREA_W, MAP_AREA_H);

	/* 网格 */
	s_lineColor = MAP_COLOR_GRID;
	MapView_DrawGrid(0);

	/* 轨迹折线 */
	s_lineColor = MAP_COLOR_TRACK;
	for (i = 0; i < cnt; i++)
	{
		MapView_Project(pts[i].latE7, pts[i].lonE7, &ax, &ay);

		if (havePrev) MapView_LineLcd(px, py, ax, ay);
		px = ax;
		py = ay;
		havePrev = 1;
	}

	/* 指北箭头 + 比例尺 */
	s_lineColor = LIGHT_CYAN;
	MapView_DrawNorth(0);
	MapView_DrawScaleBar(0);

	/* 当前位置(红点); 轨迹为空时也显示, 方便确认定位是否在动 */
	{
		Location_DataTypeDef *loc = Location_GetData();
		int nxp, nyp;

		MapView_Project(loc->latE7, loc->lonE7, &nxp, &nyp);
		if (MapView_InArea(nxp, nyp))
		{
			LCD_SetColor(MAP_COLOR_NOW);
			LCD_FillCircle((uint16_t)(MAP_AREA_X + nxp), (uint16_t)(MAP_AREA_Y + nyp), 3);
		}
	}

	/* 起点(青色小方块) */
	if (cnt > 0)
	{
		int sxp, syp;

		MapView_Project(pts[0].latE7, pts[0].lonE7, &sxp, &syp);
		if (MapView_InArea(sxp, syp))
		{
			LCD_SetColor(MAP_COLOR_START);
			LCD_FillRect((uint16_t)(MAP_AREA_X + sxp - 2), (uint16_t)(MAP_AREA_Y + syp - 2), 5, 5);
		}
	}

	/* 区域边框 */
	s_lineColor = MAP_COLOR_FRAME;
	MapView_LineLcd(0, 0, MAP_AREA_W - 1, 0);
	MapView_LineLcd(0, MAP_AREA_H - 1, MAP_AREA_W - 1, MAP_AREA_H - 1);
	MapView_LineLcd(0, 0, 0, MAP_AREA_H - 1);
	MapView_LineLcd(MAP_AREA_W - 1, 0, MAP_AREA_W - 1, MAP_AREA_H - 1);
}

/*---------------------- 位图渲染 ----------------------------*/

static void MapView_RenderBmp(void)
{
	const NavPoint_TypeDef *pts = NavLog_GetPoints();
	uint32_t cnt = NavLog_GetCount();
	uint32_t i;
	int ax, ay, px = 0, py = 0;
	uint8_t havePrev = 0;

	/* 清空(0=背景) */
	for (i = 0; i < (uint32_t)sizeof(s_bits); i++) s_bits[i] = 0;

	MapView_DrawGrid(1);

	for (i = 0; i < cnt; i++)
	{
		MapView_Project(pts[i].latE7, pts[i].lonE7, &ax, &ay);

		if (havePrev) MapView_LineBmp(px, py, ax, ay, 1);
		px = ax;
		py = ay;
		havePrev = 1;
	}

	if (cnt > 0)
	{
		int sxp, syp;
		MapView_Project(pts[0].latE7, pts[0].lonE7, &sxp, &syp);
		MapView_FillBmp(sxp, syp, 2);
	}

	{
		Location_DataTypeDef *loc = Location_GetData();
		int nxp, nyp;

		MapView_Project(loc->latE7, loc->lonE7, &nxp, &nyp);
		MapView_FillBmp(nxp, nyp, 3);
	}

	MapView_DrawNorth(1);
	MapView_DrawScaleBar(1);
}

/*---------------------- 对外接口 ----------------------------*/

void MapView_Init(void)
{
	s_zoom        = 1.0f;
	s_forceRedraw = 1;
	s_wasVisible  = 0;
	s_drawnCount  = 0xFFFFFFFF;
	s_orgLatE7    = 0;
	s_orgLonE7    = 0;
	MapView_RecalcFit();
}

void MapView_Reset(void)
{
	s_zoom        = 1.0f;
	s_forceRedraw = 1;
}

void MapView_ZoomIn(void)
{
	s_zoom *= MAP_ZOOM_STEP;
	if (s_zoom > MAP_ZOOM_MAX) s_zoom = MAP_ZOOM_MAX;
	s_forceRedraw = 1;
}

void MapView_ZoomOut(void)
{
	s_zoom /= MAP_ZOOM_STEP;
	if (s_zoom < MAP_ZOOM_MIN) s_zoom = MAP_ZOOM_MIN;
	s_forceRedraw = 1;
}

void MapView_NotifyCleared(void)
{
	s_forceRedraw = 1;
	s_zoom        = 1.0f;
}

float MapView_GetGridM(void)
{
	return s_gridM;
}

float MapView_GetZoom(void)
{
	return s_zoom;
}

/*************************************************************************************************
*	函 数 名:	MapView_Refresh
*
*	入口参数:	visible - 地图页当前是否正在显示(1=显示)
*
*	函数功能:	每300ms调用一次。只有在地图页可见、并且数据真的变化时才重绘,
*				其它页面显示时完全不动屏幕(否则会把别人的页面画花)
*
*************************************************************************************************/
void MapView_Refresh(uint8_t visible)
{
	const NavPoint_TypeDef *pts;
	uint32_t cnt;
	int32_t  lastLat = 0, lastLon = 0;
	uint8_t  changed;

	if (!visible)
	{
		s_wasVisible = 0;
		return;
	}

	cnt = NavLog_GetCount();
	pts = NavLog_GetPoints();
	if (cnt > 0)
	{
		lastLat = pts[cnt - 1].latE7;
		lastLon = pts[cnt - 1].lonE7;
	}

	changed = (uint8_t)(s_forceRedraw ||
	                    !s_wasVisible ||
	                    cnt != s_drawnCount ||
	                    lastLat != s_drawnLatE7 ||
	                    lastLon != s_drawnLonE7);

	if (!changed) return;

	MapView_RecalcFit();
	MapView_DrawLcd();

	s_drawnCount  = cnt;
	s_drawnLatE7  = lastLat;
	s_drawnLonE7  = lastLon;
	s_wasVisible  = 1;
	s_forceRedraw = 0;
}

/*************************************************************************************************
*	函 数 名:	MapView_SaveToSd
*
*	返 回 值:	1 - 保存成功;  0 - 无SD卡或写入失败
*
*	函数功能:	按当前视图把地图渲染成1bpp位图 -> 写成 24位BMP 到SD卡,
*				并把轨迹CSV刷盘
*
*************************************************************************************************/
uint8_t MapView_SaveToSd(void)
{
	if (!NavLog_IsSdOk())
	{
		return 0;
	}

	MapView_RecalcFit();
	MapView_RenderBmp();

	if (!NavLog_WriteBmp1bpp(s_bits, MAP_AREA_W, MAP_AREA_H, MAP_BMP_FG, MAP_BMP_BG))
	{
		return 0;
	}

	NavLog_Sync();		// 轨迹CSV一并落盘, 保证BMP和CSV是同一次的状态
	return 1;
}
