/**
  ******************************************************************************
  * @file    bsp.h
  * @brief   Board Support Package public API.
  *
  * The application layer should include this header for board start-up and
  * for board-level status queries. Device/algorithm modules keep their own
  * headers (car.h, gps.h, location.h, ...) and are called through those.
  ******************************************************************************
  */

#ifndef __BSP_H
#define __BSP_H

#include "stm32f4xx.h"

/**
  * @brief  Initialise the board: clocks/time base, communication ports,
  *         chassis, sensors, LCD, GPS and navigation middleware.
  * @note   Call once after reset, before the main loop.
  */
void BSP_Init(void);

/**
  * @brief  MPU6050 presence after BSP_Init().
  * @retval 1: present, 0: not found
  */
uint8_t BSP_IsMpuReady(void);
/**
  * @brief  Optional independent watchdog (IWDG). Disabled by default.
  * @note   Enable with APP_USE_IWDG=1 in bsp.c, then call App_IwdgFeed()
  *         periodically from a task that is guaranteed to keep running
  *         (the LED heartbeat is a good place).
  */
void App_IwdgInit(void);
void App_IwdgFeed(void);

#endif /* __BSP_H */
