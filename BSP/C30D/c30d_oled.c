#include "c30d_oled.h"
#include "c30d_soft_i2c.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include <string.h>

#define DC (1U << 11)
#define RST (1U << 12)
#define DIN (1U << 13)
#define CLK (1U << 14)
uint8_t c30d_oled_framebuffer[8][128];
/* Compact 5x7 column glyphs: digits followed by uppercase alphabet. */
static const uint8_t glyphs[36][5] = {
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, {0, 0x42, 0x7f, 0x40, 0}, {0x42, 0x61, 0x51, 0x49, 0x46}, {0x21, 0x41, 0x45, 0x4b, 0x31}, {0x18, 0x14, 0x12, 0x7f, 0x10}, {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3c, 0x4a, 0x49, 0x49, 0x30}, {1, 0x71, 9, 5, 3}, {0x36, 0x49, 0x49, 0x49, 0x36}, {6, 0x49, 0x49, 0x29, 0x1e}, {0x7e, 0x11, 0x11, 0x11, 0x7e}, {0x7f, 0x49, 0x49, 0x49, 0x36}, {0x3e, 0x41, 0x41, 0x41, 0x22}, {0x7f, 0x41, 0x41, 0x22, 0x1c}, {0x7f, 0x49, 0x49, 0x49, 0x41}, {0x7f, 9, 9, 9, 1}, {0x3e, 0x41, 0x49, 0x49, 0x7a}, {0x7f, 8, 8, 8, 0x7f}, {0, 0x41, 0x7f, 0x41, 0}, {0x20, 0x40, 0x41, 0x3f, 1}, {0x7f, 8, 0x14, 0x22, 0x41}, {0x7f, 0x40, 0x40, 0x40, 0x40}, {0x7f, 2, 0x0c, 2, 0x7f}, {0x7f, 4, 8, 0x10, 0x7f}, {0x3e, 0x41, 0x41, 0x41, 0x3e}, {0x7f, 9, 9, 9, 6}, {0x3e, 0x41, 0x51, 0x21, 0x5e}, {0x7f, 9, 0x19, 0x29, 0x46}, {0x46, 0x49, 0x49, 0x49, 0x31}, {1, 1, 0x7f, 1, 1}, {0x3f, 0x40, 0x40, 0x40, 0x3f}, {0x1f, 0x20, 0x40, 0x20, 0x1f}, {0x3f, 0x40, 0x38, 0x40, 0x3f}, {0x63, 0x14, 8, 0x14, 0x63}, {7, 8, 0x70, 8, 7}, {0x61, 0x51, 0x49, 0x45, 0x43}};
static void delay_clock(void)
{
    uint32_t start = C30D_Cycles();
    while (C30D_ElapsedUs(start) < 1U)
    {
    }
}
static void byte(uint8_t data, int is_data)
{
    GPIOD->BSRR = is_data ? DC : DC << 16;
    for (unsigned i = 0; i < 8U; ++i)
    {
        GPIOD->BSRR = CLK << 16;
        GPIOD->BSRR = (data & 0x80U) != 0U ? DIN : DIN << 16;
        delay_clock();
        GPIOD->BSRR = CLK;
        delay_clock();
        data <<= 1;
    }
}
void C30D_OledInit(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIOD->BSRR = (RST | CLK | DIN | DC) << 16;
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = RST | CLK | DIN | DC;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);
    (void)osDelay(100U);
    GPIOD->BSRR = RST;
    (void)osDelay(10U);
    /* WHEELTEC C30D OLED example: 128x64 SSD1306-compatible page commands,
     * A1/C0 scan directions. No unrelated RTC/backup-domain modifications. */
    static const uint8_t init[] = {0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0, 0x40, 0x8d, 0x14,
                                   0x20, 2, 0xa1, 0xc0, 0xda, 0x12, 0x81, 0x7f, 0xd9, 0xf1, 0xdb, 0x30, 0xa4, 0xa6};
    for (unsigned i = 0; i < sizeof(init); ++i)
    {
        byte(init[i], 0);
    }
    memset(c30d_oled_framebuffer, 0, sizeof(c30d_oled_framebuffer));
    C30D_OledRefresh();
    byte(0xaf, 0);
}
void C30D_OledLine(unsigned row, const char *text)
{
    if (row >= 8U || text == NULL)
    {
        return;
    }
    memset(c30d_oled_framebuffer[row], 0, 128);
    for (unsigned col = 0; col < 21U && text[col] != '\0'; ++col)
    {
        unsigned char ch = (unsigned char)text[col];
        uint8_t symbol[5] = {0};
        const uint8_t *glyph = symbol;
        if (ch >= '0' && ch <= '9')
        {
            glyph = glyphs[ch - '0'];
        }
        else if (ch >= 'A' && ch <= 'Z')
        {
            glyph = glyphs[10U + ch - 'A'];
        }
        else if (ch == '-')
        {
            memset(symbol, 8, 5);
        }
        else if (ch == '+')
        {
            memset(symbol, 8, 5);
            symbol[2] = 0x3e;
        }
        else if (ch == '.')
        {
            symbol[2] = 0x60;
        }
        else if (ch == ':')
        {
            symbol[2] = 0x36;
        }
        else if (ch == '/')
        {
            symbol[0] = 0x20;
            symbol[1] = 0x10;
            symbol[2] = 8;
            symbol[3] = 4;
            symbol[4] = 2;
        }
        /* Match the vendor's A1/C0 display orientation: reverse page and bits. */
        for (unsigned x = 0; x < 5U; ++x)
        {
            uint8_t reversed = 0;
            for (unsigned bit = 0; bit < 8U; ++bit)
            {
                reversed |= (uint8_t)(((glyph[x] >> bit) & 1U) << (7U - bit));
            }
            c30d_oled_framebuffer[row][col * 6U + x] = reversed;
        }
    }
}
void C30D_OledRefresh(void)
{
    for (unsigned row = 0; row < 8U; ++row)
    {
        byte((uint8_t)(0xb0U + 7U - row), 0);
        byte(0, 0);
        byte(0x10, 0);
        for (unsigned x = 0; x < 128U; ++x)
        {
            byte(c30d_oled_framebuffer[row][x], 1);
        }
    }
}
