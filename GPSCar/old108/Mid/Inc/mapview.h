#ifndef __MAPVIEW_H
#define __MAPVIEW_H

#include "stm32f4xx.h"

/*=================================================================================================
 * 地图显示 / 地图导出
 *-------------------------------------------------------------------------------------------------
 * 1. 把 navlog 记录的GPS轨迹点投影成"北朝上"的平面图, 画在LCD指定区域内:
 *      - 自动按轨迹范围缩放并居中(带15%边距), 也可以用 K1/K2 手动缩放
 *      - 网格间距自动取 0.5/1/2/5/10/20/50/... 米中"像素跨度>=32"的档位
 *      - 起点(青色方块)、当前位置(红色圆点)、指北箭头、比例尺
 * 2. 同一套投影还能把地图渲染成 1bpp 位图交给 navlog 写成 24位BMP存到SD卡
 *
 * 内存: 1bpp位图缓冲 (236+7)/8 * 192 = 5760 字节(静态)
 *===============================================================================================*/

/*---------------------- LCD地图区域(避开页眉/信息行/页脚) --------*/
#define MAP_AREA_X		2
#define MAP_AREA_Y		18
#define MAP_AREA_W		236
#define MAP_AREA_H		192

/*---------------------- 缩放范围 ----------------------------*/
#define MAP_ZOOM_MIN	0.25f	// 最小放大倍数(视野最大)
#define MAP_ZOOM_MAX	8.00f	// 最大放大倍数(视野最小)
#define MAP_ZOOM_STEP	2.00f	// 每次按键的缩放步进
#define MAP_MIN_SPAN_M	20.0f	// 自动缩放时视野至少覆盖多少米(轨迹很短时不会放大到抖动可见)

/*---------------------- 接口 ----------------------------*/

void    MapView_Init(void);					// 初始化(清轨迹视图状态)
void    MapView_Reset(void);				// 复位缩放为自动
void    MapView_ZoomIn(void);				// 放大(视野变小)
void    MapView_ZoomOut(void);				// 缩小(视野变大)
void    MapView_NotifyCleared(void);		// 轨迹被清空后调用(强制重绘)
void    MapView_Refresh(uint8_t visible);	// 每300ms调用: 页面可见且数据有变化时才重绘

/*
 * 把当前地图渲染成1bpp位图并写成 24位BMP 到SD卡(0:/CARLOG/MAP.BMP),
 * 同时把CSV刷盘。返回 1=成功。
 * 说明: 写入约136KB, 期间界面会停顿一两百毫秒(用户按键触发, 可接受)。
 */
uint8_t MapView_SaveToSd(void);

float   MapView_GetGridM(void);				// 当前网格间距(m), 供页脚显示
float   MapView_GetZoom(void);				// 当前缩放倍数

#endif /* __MAPVIEW_H */
