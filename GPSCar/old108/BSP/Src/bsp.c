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
#include "asrpro.h"
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
    Delay_Init();           /* SysTick millisecond delay                    */
    Timer_Init();           /* TIM2 microsecond time base                   */

    /* ---- 2. debug and communication ------------------------------------ */
    LED_Init();             /* on-board LED (PC13)                          */
    Usart_Config();         /* USART1 debug console (PA9/PA10)              */
    K230_Init();            /* USART1 RX for K230 vision frames             */
    BT_Init();              /* USART2 Bluetooth / WiFi (PD5/PD6)            */
    ASR_Init();             /* USART3 ASRPRO voice module (PD8/PD9)         */

    /* ---- 3. chassis and sensors ---------------------------------------- */
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
    MapView_Init();         /* track map view                               */
}

/**
  * @brief  Report whether the MPU6050 answered during BSP_Init().
  * @retval 1: present, 0: not found
  */
uint8_t BSP_IsMpuReady(void)
{
    return s_mpuReady;
}
