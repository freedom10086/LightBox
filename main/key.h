#ifndef KEY_H
#define KEY_H

#include "esp_types.h"
#include "esp_event.h"
#include "io_config.h"

ESP_EVENT_DECLARE_BASE(KEY_CLICK_EVENT);

#define KEY_1_NUM GPIO_KEY_0
#define KEY_2_NUM GPIO_KEY_1

/**
 * key click event
 */
typedef enum {
    KEY_1_SHORT_CLICK,
    KEY_1_LONG_CLICK,
    KEY_2_SHORT_CLICK,
    KEY_2_LONG_CLICK,
} key_event_id_t;

void key_init();

#endif