#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "yg_display.h"

static const char *TAG = "yg";

#define EVENT_TIMER      (1 << 0)

static TaskHandle_t runloop_task_handle = NULL;
void yg_runloop_init(void);
void timer_callback(void* arg);
void runloop_task(void* arg);

void app_main(void) {
    ESP_LOGI(TAG, "nvs_flash_init");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "yg_display_init");
    yg_display_init();
    ESP_LOGI(TAG, "yg_runloop_init");
    yg_runloop_init();
}

void yg_runloop_init(void) {
    xTaskCreatePinnedToCore(runloop_task, "runloop_task", 4096, NULL, 5, &runloop_task_handle, 1);

    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_ISR,
        .name = "runloop_timer"
    };

    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, 16666)); // 16.666 ms
}

void runloop_task(void* arg) {
    uint32_t notification_value;

    while (1) {
        BaseType_t notified = xTaskNotifyWait(0x00, UINT32_MAX, &notification_value, portMAX_DELAY);

        // No notification received
        if (notified != pdTRUE) { continue; }
        // No matching bits set
        if (!(notification_value & EVENT_TIMER)) { continue;}
        
        // Handle touches
        esp_lcd_touch_handle_t touch = NULL;
        if (yg_display_get_touches(&touch) && touch != NULL) {
            uint16_t touch_x[1];
            uint16_t touch_y[1];
            uint16_t touch_strength[1];
            uint8_t touch_cnt = 0;

            bool touchpad_pressed = esp_lcd_touch_get_coordinates(touch, touch_x, touch_y, touch_strength, &touch_cnt, 1);
            if (touchpad_pressed && touch_cnt > 0) {
                ESP_LOGI(TAG, "Touchpad pressed at x: %d, y: %d, strength: %d", touch_x[0], touch_y[0], touch_strength[0]);
            }
        }
    }
}

void timer_callback(void* arg) {
    BaseType_t woken = false;
    xTaskNotifyFromISR(runloop_task_handle, EVENT_TIMER, eSetBits, &woken);
    if (woken) { portYIELD_FROM_ISR(); }
}
