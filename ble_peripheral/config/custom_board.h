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
#ifndef REDBEARNANO_H
#define REDBEARNANO_H

// LEDs definitions for RedBear Nano 1.5
#define LEDS_NUMBER    4

#define LED_START      19
#define LED_1          19
#define LED_2          19
#define LED_STOP       19

#define LEDS_LIST { LED_1 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)

#define LEDS_MASK      (BSP_LED_0_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 1

#define BUTTON_START   29
#define BUTTON_1       29
#define BUTTON_STOP    29
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)

#define BUTTONS_MASK   BSP_BUTTON_0_MASK

#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
#define HWFC           true

#define SPIS_MISO_PIN  9    // SPI MISO signal.
#define SPIS_CSN_PIN   10    // SPI CSN signal.
#define SPIS_MOSI_PIN  11    // SPI MOSI signal.
#define SPIS_SCK_PIN   8    // SPI SCK signal.

// serialization APPLICATION board
#define SER_CONN_CHIP_RESET_PIN     12    // Pin used to reset connectivity chip

#define SER_APP_RX_PIN              11   // UART RX pin number.
#define SER_APP_TX_PIN              9    // UART TX pin number.
#define SER_APP_CTS_PIN             10   // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             8    // UART Request To Send pin number.

#define SER_APP_SPIM0_SCK_PIN       7     // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      0     // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      30    // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        25    // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       29    // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       28    // SPI REQUEST GPIO pin number

// Low frequency clock source to be used by the SoftDevice
#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#endif

#endif // PCA10028_H
