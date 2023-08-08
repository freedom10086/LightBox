//
// Created by yang on 2023/8/1.
//

#ifndef ESP32C6_HELLO_WS2812_H
#define ESP32C6_HELLO_WS2812_H

#include "fonts.h"

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))

typedef struct {
    uint8_t blue;
    uint8_t green;
    uint8_t red;
} __attribute__((packed)) ws2812_pixel_t;

void ws2812_init(void);

void ws2812_power_on();

void ws2812_power_off();

void ws2812_power_toggle();

void ws2812_set_pixel(uint8_t x, uint8_t y, ws2812_pixel_t pixel);

void ws2812_set_color(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b);

void ws2812_draw_line(int x0, int y0, int x1, int y1, ws2812_pixel_t color);

void ws2812_draw_horizontal_line(int x, int y, int width, ws2812_pixel_t color);

void ws2812_draw_vertical_line(int x, int y, int height, ws2812_pixel_t color);

void ws2812_draw_rectangle(int x0, int y0, int x1, int y1, ws2812_pixel_t color);

void ws2812_draw_filled_rectangle(int x0, int y0, int x1, int y1, ws2812_pixel_t color);

void ws2812_draw_circle(int x, int y, int radius, ws2812_pixel_t color);

void ws2812_draw_filled_circle(int x, int y, int radius, ws2812_pixel_t color);

void ws2812_draw_char_at(int x, int y, char ascii_char, sFONT *font, ws2812_pixel_t color);

void ws2812_draw_chinese_char_at(int x, int y, uint16_t font_char, sFONT *font, ws2812_pixel_t color);

void ws2812_clear_all();

void ws2812_refresh();

void ws2812_deinit(void);

#endif //ESP32C6_HELLO_WS2812_H
