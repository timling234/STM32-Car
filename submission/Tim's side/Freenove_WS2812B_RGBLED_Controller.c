/* Freenove_WS2812B_RGBLED_Controller.c */

#include "Freenove_WS2812B_RGBLED_Controller.h"

static uint8_t writeReg(Freenove_WS2812B_Controller *ctrl, uint8_t cmd, uint8_t *data, uint8_t size)
{
    uint8_t buffer[32];
    buffer[0] = cmd;
    for (uint8_t i = 0; i < size; i++)
        buffer[i + 1] = data[i];

    return HAL_I2C_Master_Transmit(ctrl->i2cHandle, ctrl->address << 1, buffer, size + 1, HAL_MAX_DELAY);
}

static uint8_t writeRegOneByte(Freenove_WS2812B_Controller *ctrl, uint8_t val)
{
    return HAL_I2C_Master_Transmit(ctrl->i2cHandle, ctrl->address << 1, &val, 1, HAL_MAX_DELAY);
}

void Freenove_WS2812B_Init(Freenove_WS2812B_Controller *ctrl, I2C_HandleTypeDef *hi2c, uint8_t address, uint16_t ledCount, uint8_t type)
{
    ctrl->i2cHandle = hi2c;
    ctrl->address = address;
    ctrl->ledCount = ledCount;
    ctrl->rOffset = (type >> 4) & 0x03;
    ctrl->gOffset = (type >> 2) & 0x03;
    ctrl->bOffset = type & 0x03;
}

uint8_t Freenove_WS2812B_Begin(Freenove_WS2812B_Controller *ctrl)
{
    uint8_t count = ctrl->ledCount;
    return writeReg(ctrl, REG_LEDS_COUNTS, &count, 1);
}

uint8_t Freenove_WS2812B_SetLedColor(Freenove_WS2812B_Controller *ctrl, uint8_t index, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t colors[3];
    colors[ctrl->rOffset] = r;
    colors[ctrl->gOffset] = g;
    colors[ctrl->bOffset] = b;

    uint8_t data[4] = { index, colors[0], colors[1], colors[2] };
    return writeReg(ctrl, REG_SET_LED_COLOR, data, 4);
}

uint8_t Freenove_WS2812B_SetAllLedsColor(Freenove_WS2812B_Controller *ctrl, uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t colors[3];
    colors[ctrl->rOffset] = r;
    colors[ctrl->gOffset] = g;
    colors[ctrl->bOffset] = b;

    return writeReg(ctrl, REG_SET_ALL_LEDS_COLOR, colors, 3);
}

uint8_t Freenove_WS2812B_Show(Freenove_WS2812B_Controller *ctrl)
{
    return writeRegOneByte(ctrl, REG_TRANS_DATA_TO_LED);
}

uint32_t Freenove_WS2812B_Wheel(uint8_t pos)
{
    uint32_t WheelPos = pos % 255;
    if (WheelPos < 85)
    {
        return ((255 - WheelPos * 3) << 16) | ((WheelPos * 3) << 8);
    }
    else if (WheelPos < 170)
    {
        WheelPos -= 85;
        return ((255 - WheelPos * 3) << 8) | (WheelPos * 3);
    }
    else
    {
        WheelPos -= 170;
        return (WheelPos * 3) << 16 | (255 - WheelPos * 3);
    }
}
