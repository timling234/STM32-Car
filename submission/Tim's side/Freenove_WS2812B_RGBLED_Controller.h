/* Freenove_WS2812B_RGBLED_Controller.h */

#ifndef _FREENOVE_WS2812B_RGBLED_CONTROLLER_H_
#define _FREENOVE_WS2812B_RGBLED_CONTROLLER_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

#define REG_LEDS_COUNTS                 0
#define REG_SET_LED_COLOR_DATA         1
#define REG_SET_LED_COLOR              2
#define REG_SET_ALL_LEDS_COLOR_DATA    3
#define REG_SET_ALL_LEDS_COLOR         4
#define REG_TRANS_DATA_TO_LED          5

#define REG_LEDS_COUNT_READ            0xFA

#define TYPE_RGB   0x06
#define TYPE_RBG   0x09
#define TYPE_GRB   0x12
#define TYPE_GBR   0x21
#define TYPE_BRG   0x18
#define TYPE_BGR   0x24

typedef struct {
    I2C_HandleTypeDef *i2cHandle;
    uint8_t address;
    uint16_t ledCount;
    uint8_t rOffset;
    uint8_t gOffset;
    uint8_t bOffset;
} Freenove_WS2812B_Controller;

void Freenove_WS2812B_Init(Freenove_WS2812B_Controller *controller, I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t ledCount, uint8_t type);
uint8_t Freenove_WS2812B_Begin(Freenove_WS2812B_Controller *controller);
uint8_t Freenove_WS2812B_SetLedColor(Freenove_WS2812B_Controller *controller, uint8_t index, uint8_t r, uint8_t g, uint8_t b);
uint8_t Freenove_WS2812B_SetAllLedsColor(Freenove_WS2812B_Controller *controller, uint8_t r, uint8_t g, uint8_t b);
uint8_t Freenove_WS2812B_Show(Freenove_WS2812B_Controller *controller);
uint32_t Freenove_WS2812B_Wheel(uint8_t pos);

#endif /* _FREENOVE_WS2812B_RGBLED_CONTROLLER_H_ */
