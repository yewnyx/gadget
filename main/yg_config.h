#pragma once

#include "spleen-6x12.h"

#define YG_LCD_H_RES              240
#define YG_LCD_V_RES              240
#define YG_LCD_FILL_BLOCKS        10
#define YG_LCD_BUF_PIXEL_COUNT    YG_LCD_H_RES * YG_LCD_V_RES / YG_LCD_FILL_BLOCKS
#define YG_LCD_BUF_SIZE           YG_LCD_BUF_PIXEL_COUNT * sizeof(uint16_t)
#define YG_LCD_LINE_WIDTH         YG_LCD_H_RES / FONT_WIDTH
#define YG_LCD_LINE_COUNT         YG_LCD_V_RES / FONT_HEIGHT
#define YG_LCD_CHAR_COUNT         YG_LCD_LINE_WIDTH * YG_LCD_LINE_COUNT
#define YG_LCD_CHAR_SIZE          (FONT_WIDTH * FONT_HEIGHT / sizeof(uint8_t))

#define YG_LCD_CMD_BITS           8
#define YG_LCD_PARAM_BITS         8
#define YG_LCD_SPI_HOST           SPI2_HOST

#define YG_LCD_PIXEL_CLOCK_HZ     (80 * 1000 * 1000)
#define YG_LCD_BK_LIGHT_ON_LEVEL  1
#define YG_LCD_BK_LIGHT_OFF_LEVEL !YG_LCD_BK_LIGHT_ON_LEVEL

#define YG_PIN_NUM_LCD_BL         GPIO_NUM_2
#define YG_PIN_NUM_TP_INT         GPIO_NUM_5
#define YG_PIN_NUM_TP_SDA         GPIO_NUM_6
#define YG_PIN_NUM_TP_SCL         GPIO_NUM_7
#define YG_PIN_NUM_LCD_DC         GPIO_NUM_8
#define YG_PIN_NUM_LCD_CS         GPIO_NUM_9
#define YG_PIN_NUM_LCD_SCLK       GPIO_NUM_10
#define YG_PIN_NUM_LCD_MOSI       GPIO_NUM_11
#define YG_PIN_NUM_LCD_MISO       GPIO_NUM_12
#define YG_PIN_NUM_TP_RST         GPIO_NUM_13
#define YG_PIN_NUM_LCD_RST        GPIO_NUM_14