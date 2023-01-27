/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef MAIN_HWCONF_MOXIE_TEST_H_
#define MAIN_HWCONF_MOXIE_TEST_H_

#include "driver/gpio.h"

#define HW_NAME						"Moxie Express Logger"

#define HW_INIT_HOOK()				hw_init()

// LEDs
// #define LED_ADDRESSABLE_PIN          9
#define LED_RED_PIN					9
#define LED_BLUE_PIN				none

// replace individual red/blue leds with 2x addressable leds to save gpio
#define LED_RED_ON()				gpio_set_level(LED_RED_PIN, 1)
#define LED_RED_OFF()				gpio_set_level(LED_RED_PIN, 0)

#define LED_BLUE_ON()				//gpio_set_level(LED_BLUE_PIN, 1)
#define LED_BLUE_OFF()				//gpio_set_level(LED_BLUE_PIN, 0)


// all pin #'s accurate to moxie drive v2 hardware
// CAN
#define CAN_TX_GPIO_NUM				10 
#define CAN_RX_GPIO_NUM				5

// mneed to check if stcard pins fine 
// being strapped high when inserted/powered
// SD-card
#define SD_PIN_MOSI					6
#define SD_PIN_MISO					2
#define SD_PIN_SCK					8
#define SD_PIN_CS					7

// SWD
#define STM23_SWDIO                 3
#define STM23_SWCLK                 4

// I2C
// #define I2C_SDA                     3
// #define I2C_SCL                     4

// GPS_UART
#define GPS_UART_NUM					0
#define GPS_UART_BAUDRATE				115200
#define GPS_UART_TX						21
#define GPS_UART_RX						20

// off the shelf gps units are 9600 baud by default.
// try 9600 baud first then 115200 baud if fails?
// try to configure for 115200 baud for that 1 gps that doesn't have nonvolatile memory?


// VESC_UART
#define VESC_UART_NUM					0
#define VESC_UART_BAUDRATE				115200
#define VESC_UART_TX						21
#define VESC_UART_RX						20

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_MOXIE_TEST_H_ */
