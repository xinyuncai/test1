/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef BOARDS_H
#define BOARDS_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "nordic_common.h"

#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"

#ifdef FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "timers.h"
#endif

#include "nrf.h"
#include "nrf_log.h"

#include "app_error.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "app_trace.h"

#include "device_manager.h"
#include "pstorage.h"
#include "softdevice_handler.h"

#include "gu620.h"
#include "adxl345.h"
#include "remove_dec.h"
#include "rtc.h"
#include "bat_level_dec.h"
#include "device_info.h"

#include "cfs.h"
#include "cfs-coffee.h"

#define LED_5                       26
#define LED_6                       28
#define WORK_LED_RED                LED_5
#define WORK_LED_BLUE               LED_6
#define WORK_LED_RED_MASK           (1<<WORK_LED_RED)
#define WORK_LED_BLUE_MASK          (1<<WORK_LED_BLUE)
#define LEDS_ON(leds_mask)          NRF_GPIO->OUTSET = leds_mask
#define LEDS_OFF(leds_mask)         NRF_GPIO->OUTCLR = leds_mask
#define LEDS_INVERT(leds_mask)      do { uint32_t gpio_state = NRF_GPIO->OUT;       \
                                    NRF_GPIO->OUTSET = ((leds_mask) & ~gpio_state); \
                                    NRF_GPIO->OUTCLR = ((leds_mask) & gpio_state); } while (0)

                            
#define NRF_CLOCK_LFCLKSRC          {.source        = NRF_CLOCK_LF_SRC_XTAL, \
                                     .rc_ctiv       = 0, \
                                     .rc_temp_ctiv  = 0, \
                                     .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}     


#define I2C_SCL_PIN                 6
#define I2C_SDA_PIN                 4

void bsp_init(void);
void bsp_uart_init(void);
#endif
