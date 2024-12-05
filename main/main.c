#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "nvs_flash.h"
#include "driver/i2c_master.h"

#include "yg_config.h"
#include "yg_display.h"

static const char *TAG = "yg";

#define EVENT_TIMER      (1 << 0)

static TaskHandle_t runloop_task_handle = NULL;
void yg_runloop_init(void);
void timer_callback(void* arg);
void runloop_task(void* arg);

void app_main(void) {
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << YG_PIN_NUM_LCD_BL,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));


    ESP_LOGI(TAG, "nvs_flash_init");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "i2c_new_master_bus");
    const i2c_master_bus_config_t i2c_bus_conf = {
        .i2c_port = 0,
        .sda_io_num = YG_PIN_NUM_TP_SDA,
        .scl_io_num = YG_PIN_NUM_TP_SCL,
        .flags.enable_internal_pullup = true,
        .clk_source = I2C_CLK_SRC_XTAL,
    };
    i2c_master_bus_handle_t i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_conf, &i2c_bus_handle));

    ESP_LOGI(TAG, "yg_display_init");
    yg_display_init(i2c_bus_handle);
    ESP_LOGI(TAG, "Turn on backlight");
    ESP_ERROR_CHECK(gpio_set_level(YG_PIN_NUM_LCD_BL, YG_LCD_BK_LIGHT_ON_LEVEL));
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
