/**
  ******************************************************************************
  * @file    bsp.c
  * @brief   Board Support Package: board-level bring-up / external API entry.
  *
  * Layer rule:
  *   APP (MY)  -> BSP -> MID (algorithms) -> PROGRAM (module drivers) -> DRIVE
  *
  * BSP only contains board-level orchestration. It calls the public init APIs
  * exposed by DRIVE / PROGRAM / MID, so the application does not need to know
  * the concrete driver order. When porting to another board or to an RTOS,
  * this is the only file that has to be adapted for start-up ordering.
  ******************************************************************************
  */

#include "bsp.h"

#include "delay.h"
#include "timer.h"
#include "led.h"
#include "usart.h"

#include "K230.h"
#include "bluetooth.h"
#include "lidar.h"
#include "estop.h"
#include "uart_host.h"
#include "car.h"

#include "sensor_adc.h"
#include "servo.h"
#include "ultrasonic.h"
#include "buzzer.h"
#include "car_led.h"
#include "car_key.h"
#include "mpu6050.h"
#include "lcd_spi_154.h"
#include "gps.h"

#include "location.h"
#include "navlog.h"
#include "mapview.h"
#include "map_collent.h"

/* Result of the MPU6050 probe, kept inside BSP so this file does not depend
 * on application globals. The application reads it through BSP_IsMpuReady(). */
static uint8_t s_mpuReady = 0;

/**
  * @brief  Initialise all on-board hardware and external modules.
  * @note   The order is intentional:
  *         1. time base (Delay/Timer)
  *         2. debug + communication ports
  *         3. actuators / sensors
  *         4. display, GPS and the navigation middleware
  *         Delay_Init() must be first because later drivers use Delay_ms().
  */
void BSP_Init(void)
{
    /* ---- 1. time base -------------------------------------------------- */
    Delay_Init();           /* TIM2 us time base (SysTick belongs to RTOS)  */
    Timer_Init();           /* TIM2 microsecond time base                   */

    /* ---- 2. debug and communication ------------------------------------ */
    LED_Init();             /* on-board LED (PC13)                          */
    Usart_Config();         /* USART1 debug console (PA9/PA10)              */
    K230_Init();            /* USART1 RX for K230 vision frames             */
    BT_Init();              /* USART2 ESP32: Bluetooth / WiFi / voice       */
    UARTHost_Init();
    Lidar_Init();           /* USART3 LD19 lidar (PD8/PD9)                  */

    /* ---- 3. chassis and sensors ---------------------------------------- */
    Estop_Init();           /* PE0 emergency input + PB1 motor STBY         */
    Car_Init();             /* motor PWM + encoders                         */
    SensorADC_Init();       /* power / brightness / humidity ADC            */
    Servo_Init();           /* servo (PB14)                                 */
    Ultrasonic_Init();      /* front + radar ultrasonic                     */
    Buzzer_Init();          /* buzzer (PF6)                                 */
    CarLED_Init();          /* body LEDs (PG0~PG3)                          */
    CarKey_Init();          /* body keys                                    */
    s_mpuReady = (MPU6050_Init() == 0) ? 1 : 0;

    /* ---- 4. display, positioning and navigation ------------------------ */
    SPI_LCD_Init();         /* LCD                                          */
    GPS_Init();             /* USART6 GPS (PG9/PG14)                        */
    Location_Init();        /* fusion positioning, needs ~0.6 s still time  */

    NavLog_Init();          /* mount SD and open 0:/CARLOG/TRACK.CSV        */
    MapView_Init();         /* track + topology map view                    */
    MapCollent_Init();      /* 200 semantic nodes + one 64x64 local grid    */
	App_IwdgInit();         /* optional IWDG, no-op while APP_USE_IWDG == 0   */
}

/**
  * @brief  Report whether the MPU6050 answered during BSP_Init().
  * @retval 1: present, 0: not found
  */
uint8_t BSP_IsMpuReady(void)
{
    return s_mpuReady;
}
/*----------------------------------------------------------------------------
 *  Optional independent watchdog
 *----------------------------------------------------------------------------
 *  Why it is off by default:
 *    - a wrong timeout turns every debug session into a reset loop;
 *    - the current hang sources (missing USART3 IRQ, blocking ultrasonic)
 *      are already fixed, so a watchdog is a safety net, not a fix.
 *
 *  When enabled, the IWDG is clocked by LSI (30~60 kHz, so the real timeout
 *  is roughly half..double of the nominal value below) and it CANNOT be
 *  stopped once started - only a power cycle or a reset clears it.
 *
 *  Feed it from exactly ONE place (see LedTask in MY/main.c). If that task
 *  stops running (higher priority task spins forever), the MCU resets.
 *--------------------------------------------------------------------------*/
#define APP_USE_IWDG        0       /* 1 = enable, 0 = disabled (default) */
#define IWDG_PRESCALER      4       /* 0:/4 1:/8 2:/16 3:/32 4:/64 5:/128 6:/256 */
#define IWDG_RELOAD         1000    /* LSI/64 = 500 Hz -> 1000 = 2 s nominal */

void App_IwdgInit(void)
{
#if APP_USE_IWDG
	IWDG->KR  = 0x5555;                 /* unlock PR/RLR                    */
	IWDG->PR  = IWDG_PRESCALER;
	IWDG->RLR = IWDG_RELOAD;
	IWDG->KR  = 0xAAAA;                 /* load the counter                 */
	IWDG->KR  = 0xCCCC;                 /* start: cannot be stopped anymore */
#endif
}

void App_IwdgFeed(void)
{
#if APP_USE_IWDG
	IWDG->KR = 0xAAAA;
#endif
}
