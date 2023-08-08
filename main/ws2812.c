#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "ws2812_encoder.h"
#include "driver/gpio.h"

#include "ws2812.h"
#include "io_config.h"

// 10MHz resolution, 1 tick = 0.1us
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000

#define LED_H_NUMBERS           16
#define LED_V_NUMBERS           16
#define LED_NUMBERS             LED_H_NUMBERS * LED_V_NUMBERS
#define CHASE_SPEED_MS          10

static const char *TAG = "ws2812";

ws2812_pixel_t led_strip_pixels1[LED_NUMBERS];
ws2812_pixel_t led_strip_pixels2[LED_NUMBERS];

bool strip1_changed = true;
bool strip2_changed = true;

TaskHandle_t led_tsk_hdl;
rmt_channel_handle_t led_chan1 = NULL;
rmt_channel_handle_t led_chan2 = NULL;
rmt_encoder_handle_t led_encoder = NULL;
static uint8_t s_led_pwr_state = 0;

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void led_strip_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b) {
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    // RGB adjustment amount by hue
    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i) {
        case 0:
            *r = rgb_max;
            *g = rgb_min + rgb_adj;
            *b = rgb_min;
            break;
        case 1:
            *r = rgb_max - rgb_adj;
            *g = rgb_max;
            *b = rgb_min;
            break;
        case 2:
            *r = rgb_min;
            *g = rgb_max;
            *b = rgb_min + rgb_adj;
            break;
        case 3:
            *r = rgb_min;
            *g = rgb_max - rgb_adj;
            *b = rgb_max;
            break;
        case 4:
            *r = rgb_min + rgb_adj;
            *g = rgb_min;
            *b = rgb_max;
            break;
        default:
            *r = rgb_max;
            *g = rgb_min;
            *b = rgb_max - rgb_adj;
            break;
    }
}

static void led_task_entry(void *arg) {
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    uint16_t hue = 0;
    uint16_t start_rgb = 0;

    ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
            .loop_count = 0, // no transfer loop
    };
    while (1) {
        while (!s_led_pwr_state) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "Wait Led Pwr ON");
        }

        memset(led_strip_pixels1, 0, sizeof(led_strip_pixels1));
        memset(led_strip_pixels2, 0, sizeof(led_strip_pixels2));

        for (int i = 0; i < LED_NUMBERS; ++i) {
            // Build RGB pixels
            // 色相（Hue，简H），饱和度（Saturation，简S）和亮度（Value，简V）
            //led_strip_hsv2rgb(hue, 100, 20, &red, &green, &blue);

            if (i % 6 == 0) {
                led_strip_pixels1[i + 0].blue = 0;
                led_strip_pixels1[i + 1].green = 0;
                led_strip_pixels1[i + 2].red = 33;
            } else if (i % 6 == 2) {
                led_strip_pixels1[i * 3 + 0].blue = 33;
                led_strip_pixels1[i * 3 + 1].green = 0;
                led_strip_pixels1[i * 3 + 2].red = 0;
            } else if (i % 6 == 4) {
                led_strip_pixels1[i + 0].blue = 0;
                led_strip_pixels1[i + 1].green = 33;
                led_strip_pixels1[i + 2].red = 0;
            }

            if (i % 6 == 0) {
                led_strip_pixels2[i + 0].blue = 0;
                led_strip_pixels2[i + 1].green = 0;
                led_strip_pixels2[i + 2].red = 66;
            } else if (i % 6 == 2) {
                led_strip_pixels2[i + 0].blue = 66;
                led_strip_pixels2[i + 1].green = 0;
                led_strip_pixels2[i + 2].red = 0;
            } else if (i % 6 == 4) {
                led_strip_pixels2[i + 0].blue = 0;
                led_strip_pixels2[i + 1].green = 66;
                led_strip_pixels2[i + 2].red = 0;
            }
        }

        ws2812_refresh();

        vTaskDelay(pdMS_TO_TICKS(CHASE_SPEED_MS));
        vTaskDelay(pdMS_TO_TICKS(2000));
        start_rgb += 60;
    }

    vTaskDelete(NULL);
}

void ws2812_init(void) {
    ESP_LOGI(TAG, "Configure LED pwr gpio");
    gpio_config_t io_config = {
            .pin_bit_mask = (1ull << GPIO_LED_PWR),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = 0,
            .pull_down_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_config));
    ws2812_power_off();

    rmt_tx_channel_config_t tx_chan_config1 = {
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .gpio_num = GPIO_LED_IO1,
            .mem_block_symbols = 48, // increase the block size can make the LED less flickering
            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };

    ESP_LOGI(TAG, "Create RMT TX channel1");
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config1, &led_chan1));

    rmt_tx_channel_config_t tx_chan_config2 = {
            .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
            .gpio_num = GPIO_LED_IO2,
            .mem_block_symbols = 48, // increase the block size can make the LED less flickering
            .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
            .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_LOGI(TAG, "Create RMT TX channel2");
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config2, &led_chan2));

    ESP_LOGI(TAG, "Install led strip encoder");
    led_strip_encoder_config_t encoder_config = {
            .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_chan1));
    ESP_ERROR_CHECK(rmt_enable(led_chan2));

    /* Create NMEA Parser task */
    BaseType_t err = xTaskCreate(
            led_task_entry,
            "key_task",
            2048,
            NULL,
            uxTaskPriorityGet(NULL),
            &led_tsk_hdl);
    if (err != pdTRUE) {
        ESP_LOGE(TAG, "create key detect task failed");
    }
}

void ws2812_power_on() {
    gpio_set_level(GPIO_LED_PWR, 1);
    s_led_pwr_state = 1;
    ESP_LOGI(TAG, "ws2812 power on");
}

void ws2812_power_off() {
    gpio_set_level(GPIO_LED_PWR, 0);
    s_led_pwr_state = 0;
    ESP_LOGI(TAG, "ws2812 power off");
}

void ws2812_power_toggle() {
    if (s_led_pwr_state) {
        ws2812_power_off();
    } else {
        ws2812_power_on();
    }
}

void ws2812_set_pixel(uint8_t x, uint8_t y, ws2812_pixel_t pixel) {
    assert(x < LED_H_NUMBERS);
    assert(y < LED_V_NUMBERS);
    ws2812_set_color(x, y, pixel.red, pixel.green, pixel.blue);
}

void ws2812_set_color(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
    assert(x < LED_H_NUMBERS);
    assert(y < LED_V_NUMBERS);

    //y = 0,  x: 0  - 15
    //y = 1,  x: 16 - 31
    //y = 2,  x: 32 - 47
    //y = 3,  x: 48 - 63
    //y = 4,  x: 64 - 79
    //y = 5,  x: 80 - 95
    //y = 6,  x: 96 - 111
    //y = 7,  x: 112 - 127

    if (y % 2 == 0) {
        x = LED_H_NUMBERS - 1 - x;
    }
    if (y < 8) {
        led_strip_pixels1[y * LED_H_NUMBERS + x].red = r;
        led_strip_pixels1[y * LED_H_NUMBERS + x].red = g;
        led_strip_pixels1[y * LED_H_NUMBERS + x].red = b;
    } else {
        y -= 8;
        led_strip_pixels2[y * LED_H_NUMBERS + x].red = r;
        led_strip_pixels2[y * LED_H_NUMBERS + x].red = g;
        led_strip_pixels2[y * LED_H_NUMBERS + x].red = b;
    }
}

void ws2812_draw_line(int x0, int y0, int x1, int y1, ws2812_pixel_t color) {
    /* Bresenham algorithm */
    int dx = x1 - x0 >= 0 ? x1 - x0 : x0 - x1;
    int sx = x0 < x1 ? 1 : -1;
    int dy = y1 - y0 <= 0 ? y1 - y0 : y0 - y1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while ((x0 != x1) && (y0 != y1)) {
        ws2812_set_pixel(x0, y0, color);
        if (2 * err >= dy) {
            err += dy;
            x0 += sx;
        }
        if (2 * err <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void ws2812_draw_horizontal_line(int x, int y, int width, ws2812_pixel_t color) {
    int i;
    for (i = x; i < MIN(x + width, LED_H_NUMBERS); i++) {
        ws2812_set_pixel(i, y, color);
    }
}

void ws2812_draw_vertical_line(int x, int y, int height, ws2812_pixel_t color) {
    int i;
    for (i = y; i < MIN(y + height, LED_V_NUMBERS); i++) {
        ws2812_set_pixel(x, i, color);
    }
}

void ws2812_draw_rectangle(int x0, int y0, int x1, int y1, ws2812_pixel_t color) {
    int min_x, min_y, max_x, max_y;
    min_x = x1 > x0 ? x0 : x1;
    max_x = x1 > x0 ? x1 : x0;
    min_y = y1 > y0 ? y0 : y1;
    max_y = y1 > y0 ? y1 : y0;

    ws2812_draw_horizontal_line(min_x, min_y, max_x - min_x + 1, color);
    ws2812_draw_horizontal_line(min_x, max_y, max_x - min_x + 1, color);
    ws2812_draw_vertical_line(min_x, min_y, max_y - min_y + 1, color);
    ws2812_draw_vertical_line(max_x, min_y, max_y - min_y + 1, color);
}

void ws2812_draw_filled_rectangle(int x0, int y0, int x1, int y1, ws2812_pixel_t color) {
    int min_x, min_y, max_x, max_y;
    int i;
    min_x = x1 > x0 ? x0 : x1;
    max_x = x1 > x0 ? x1 : x0;
    min_y = y1 > y0 ? y0 : y1;
    max_y = y1 > y0 ? y1 : y0;

    for (i = min_x; i <= max_x; i++) {
        ws2812_draw_vertical_line(i, min_y, max_y - min_y + 1, color);
    }
}

void ws2812_draw_circle(int x, int y, int radius, ws2812_pixel_t color) {
    /* Bresenham algorithm */
    int x_pos = -radius;
    int y_pos = 0;
    int err = 2 - 2 * radius;
    int e2;

    do {
        ws2812_set_pixel(x - x_pos, y + y_pos, color);
        ws2812_set_pixel(x + x_pos, y + y_pos, color);
        ws2812_set_pixel(x + x_pos, y - y_pos, color);
        ws2812_set_pixel(x - x_pos, y - y_pos, color);
        e2 = err;
        if (e2 <= y_pos) {
            err += ++y_pos * 2 + 1;
            if (-x_pos == y_pos && e2 <= x_pos) {
                e2 = 0;
            }
        }
        if (e2 > x_pos) {
            err += ++x_pos * 2 + 1;
        }
    } while (x_pos <= 0);
}

void ws2812_draw_filled_circle(int x, int y, int radius, ws2812_pixel_t color) {
    /* Bresenham algorithm */
    int x_pos = -radius;
    int y_pos = 0;
    int err = 2 - 2 * radius;
    int e2;

    do {
        ws2812_set_pixel(x - x_pos, y + y_pos, color);
        ws2812_set_pixel(x + x_pos, y + y_pos, color);
        ws2812_set_pixel(x + x_pos, y - y_pos, color);
        ws2812_set_pixel(x - x_pos, y - y_pos, color);
        ws2812_draw_horizontal_line(x + x_pos, y + y_pos, 2 * (-x_pos) + 1, color);
        ws2812_draw_horizontal_line(x + x_pos, y - y_pos, 2 * (-x_pos) + 1, color);
        e2 = err;
        if (e2 <= y_pos) {
            err += ++y_pos * 2 + 1;
            if (-x_pos == y_pos && e2 <= x_pos) {
                e2 = 0;
            }
        }
        if (e2 > x_pos) {
            err += ++x_pos * 2 + 1;
        }
    } while (x_pos <= 0);
}

void ws2812_draw_char_at(int x, int y, char ascii_char, sFONT *font, ws2812_pixel_t color) {
    int i, j;
    unsigned int char_offset =
            (ascii_char - font->start) * font->Height * (font->Width / 8 + (font->Width % 8 ? 1 : 0));
    const uint8_t *ptr = &font->table[char_offset];

    for (j = 0; j < font->Height; j++) {
        for (i = 0; i < font->Width; i++) {
            if (*ptr & (0x80 >> (i % 8))) {
                ws2812_set_pixel(x + i, y + j, color);
            }
            if (i % 8 == 7) {
                ptr++;
            }
        }
        if (font->Width % 8 != 0) {
            ptr++;
        }
    }
}

// 0xA1A1~0xFEFE
// hzk16
void ws2812_draw_chinese_char_at(int x, int y, uint16_t font_char, sFONT *font, ws2812_pixel_t color) {
    int i, j;
    unsigned int char_offset = (94 * (unsigned int) ((font_char & 0xff) - 0xa0 - 1) + ((font_char >> 8) - 0xa0 - 1))
                               * font->Height * (font->Width / 8 + (font->Width % 8 ? 1 : 0));
    const uint8_t *ptr = &font->table[char_offset];

    for (j = 0; j < font->Height; j++) {
        for (i = 0; i < font->Width; i++) {
            if (*ptr & (0x80 >> (i % 8))) {
                ws2812_set_pixel(x + i, y + j, color);
            }
            if (i % 8 == 7) {
                ptr++;
            }
        }
        if (font->Width % 8 != 0) {
            ptr++;
        }
    }
}

void ws2812_clear_all() {
    memset(led_strip_pixels1, 0, sizeof(led_strip_pixels1));
    memset(led_strip_pixels2, 0, sizeof(led_strip_pixels2));
}

void ws2812_refresh() {
    rmt_transmit_config_t tx_config = {
            .loop_count = 0, // no transfer loop
    };

    ESP_LOGI(TAG, "LED draw");
    if (strip1_changed) {
        ESP_ERROR_CHECK(rmt_transmit(led_chan1, led_encoder,
                                     led_strip_pixels1, sizeof(led_strip_pixels1), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan1, portMAX_DELAY));
        strip1_changed = false;
    }

    if (strip2_changed) {

        ESP_ERROR_CHECK(rmt_transmit(led_chan2, led_encoder,
                                     led_strip_pixels2, sizeof(led_strip_pixels2), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan2, portMAX_DELAY));
        strip2_changed = false;
    }

    ESP_LOGI(TAG, "LED draw finish");
}

void ws2812_deinit(void) {
    if (led_tsk_hdl != NULL) {
        vTaskDelete(led_tsk_hdl);
        led_tsk_hdl = NULL;
    }

    ws2812_clear_all();
    ws2812_refresh();

    ESP_LOGI(TAG, "Disable RMT TX channel");
    rmt_disable(led_chan1);
    rmt_disable(led_chan2);

    ESP_LOGI(TAG, "UnInstall led strip encoder");
    rmt_del_encoder(led_encoder);

    ESP_LOGI(TAG, "Delete RMT TX channel1");
    rmt_del_channel(led_chan1);
    ESP_LOGI(TAG, "Delete RMT TX channel2");
    rmt_del_channel(led_chan2);
}
