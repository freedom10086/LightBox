#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "key.h"
#include "mpu6050.h"

static const char *TAG = "MAIN";

esp_event_loop_handle_t event_loop_handle;
RTC_DATA_ATTR static uint32_t boot_count = 0;

static void application_task(void *args) {
    while (1) {
        esp_err_t err = esp_event_loop_run(event_loop_handle, portMAX_DELAY);
        if (err != ESP_OK) {
            break;
        }
    }

    ESP_LOGE(TAG, "suspended task for loop %p", event_loop_handle);
    vTaskSuspend(NULL);
}

void app_main(void)
{
    boot_count++;
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
        ESP_LOGI(TAG, "wake up by cause  %d", cause);
    }

    printf("Hello world!, boot count %ld\n", boot_count);

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // create event loop
    esp_event_loop_args_t loop_args = {
            .queue_size = 16,
            .task_name = NULL // no task will be created
    };

    // Create the event loops
    ESP_ERROR_CHECK(esp_event_loop_create(&loop_args, &event_loop_handle));

    //esp_event_loop_delete(esp_gps->event_loop_hdl);

    ESP_LOGI(TAG, "starting application task");
    // Create the application task with the same priority as the current task
    xTaskCreate(application_task, "application_task", 3072, NULL, uxTaskPriorityGet(NULL), NULL);

    // key click detect init
    key_init();

    mpu_init();
}
