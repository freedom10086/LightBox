#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "driver/gpio.h"

#include "ws2812.h"
#include "io_config.h"

// 10MHz resolution, 1 tick = 0.1us
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000

#define LED_NUMBERS         128
#define CHASE_SPEED_MS      10

static const char *TAG = "ws2812";

static uint8_t led_strip_pixels1[LED_NUMBERS * 3];
static uint8_t led_strip_pixels2[LED_NUMBERS * 3];
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

        for (int i = 0; i < LED_NUMBERS; ++i) {
            // Build RGB pixels
            // 色相（Hue，简H），饱和度（Saturation，简S）和亮度（Value，简V）
            //led_strip_hsv2rgb(hue, 100, 20, &red, &green, &blue);

            if (i % 6 == 0) {
                led_strip_pixels1[i * 3 + 0] = 0;
                led_strip_pixels1[i * 3 + 1] = 0;
                led_strip_pixels1[i * 3 + 2] = 33;
            } else if (i % 6 == 1) {
                led_strip_pixels1[i * 3 + 0] = 0;
                led_strip_pixels1[i * 3 + 1] = 0;
                led_strip_pixels1[i * 3 + 2] = 0;
            } else if (i % 6 == 2) {
                led_strip_pixels1[i * 3 + 0] = 33;
                led_strip_pixels1[i * 3 + 1] = 0;
                led_strip_pixels1[i * 3 + 2] = 0;
            }  else if (i % 6 == 3) {
                led_strip_pixels1[i * 3 + 0] = 0;
                led_strip_pixels1[i * 3 + 1] = 0;
                led_strip_pixels1[i * 3 + 2] = 0;
            }  else if (i % 6 == 4) {
                led_strip_pixels1[i * 3 + 0] = 0;
                led_strip_pixels1[i * 3 + 1] = 33;
                led_strip_pixels1[i * 3 + 2] = 0;
            }  else if (i % 6 == 5) {
                led_strip_pixels1[i * 3 + 0] = 0;
                led_strip_pixels1[i * 3 + 1] = 0;
                led_strip_pixels1[i * 3 + 2] = 0;
            }

            if (i % 6 == 0) {
                led_strip_pixels2[i * 3 + 0] = 0;
                led_strip_pixels2[i * 3 + 1] = 0;
                led_strip_pixels2[i * 3 + 2] = 200;
            } else if (i % 6 == 1) {
                led_strip_pixels2[i * 3 + 0] = 0;
                led_strip_pixels2[i * 3 + 1] = 0;
                led_strip_pixels2[i * 3 + 2] = 0;
            } else if (i % 6 == 2) {
                led_strip_pixels2[i * 3 + 0] = 200;
                led_strip_pixels2[i * 3 + 1] = 0;
                led_strip_pixels2[i * 3 + 2] = 0;
            }  else if (i % 6 == 3) {
                led_strip_pixels2[i * 3 + 0] = 0;
                led_strip_pixels2[i * 3 + 1] = 0;
                led_strip_pixels2[i * 3 + 2] = 0;
            }  else if (i % 6 == 4) {
                led_strip_pixels2[i * 3 + 0] = 0;
                led_strip_pixels2[i * 3 + 1] = 200;
                led_strip_pixels2[i * 3 + 2] = 0;
            }  else if (i % 6 == 5) {
                led_strip_pixels2[i * 3 + 0] = 0;
                led_strip_pixels2[i * 3 + 1] = 0;
                led_strip_pixels2[i * 3 + 2] = 0;
            }
        }

        // Flush RGB values to LEDs
        ESP_ERROR_CHECK(rmt_transmit(led_chan1, led_encoder,
                                     led_strip_pixels1, sizeof(led_strip_pixels1), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan1, portMAX_DELAY));

        ESP_ERROR_CHECK(rmt_transmit(led_chan2, led_encoder,
                                     led_strip_pixels2, sizeof(led_strip_pixels2), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan2, portMAX_DELAY));

        ESP_LOGI(TAG, "LED draw");

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

void ws2812_deinit(void) {
    if (led_tsk_hdl != NULL) {
        vTaskDelete(led_tsk_hdl);
        led_tsk_hdl = NULL;
    }

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
