#include "yg_display.h"

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_timer.h"

#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

static const char *TAG = "yg_display";

#define YG_LCD_H_RES              240
#define YG_LCD_V_RES              240
#define YG_LCD_FILL_BLOCKS        8
#define YG_LCD_BUF_PIXEL_COUNT    YG_LCD_H_RES * YG_LCD_V_RES / YG_LCD_FILL_BLOCKS
#define YG_LCD_BUF_SIZE           YG_LCD_BUF_PIXEL_COUNT * sizeof(uint16_t)

#define YG_LCD_CMD_BITS           8
#define YG_LCD_PARAM_BITS         8
#define YG_LCD_SPI_HOST           SPI2_HOST

#define YG_LCD_PIXEL_CLOCK_HZ     (80 * 1000 * 1000)
#define YG_LCD_BK_LIGHT_ON_LEVEL  1
#define YG_LCD_BK_LIGHT_OFF_LEVEL !YG_LCD_BK_LIGHT_ON_LEVEL

#define YG_PIN_NUM_LCD_BL         GPIO_NUM_2
#define YG_PIN_NUM_TP_INT         GPIO_NUM_5
#define YG_PIN_NUM_TP_SDA         GPIO_NUM_6
#define YG_PIN_NUM_TP_SCL         GPIO_NUM_7
#define YG_PIN_NUM_LCD_DC         GPIO_NUM_8
#define YG_PIN_NUM_LCD_CS         GPIO_NUM_9
#define YG_PIN_NUM_LCD_SCLK       GPIO_NUM_10
#define YG_PIN_NUM_LCD_MOSI       GPIO_NUM_11
#define YG_PIN_NUM_LCD_MISO       GPIO_NUM_12
#define YG_PIN_NUM_TP_RST         GPIO_NUM_13
#define YG_PIN_NUM_LCD_RST        GPIO_NUM_14

typedef struct {
    uint16_t *buf1;
    uint16_t *buf2;    
    esp_lcd_panel_handle_t panel_handle;
    esp_lcd_touch_handle_t touch_handle;
    uint16_t tick_num;
    SemaphoreHandle_t transfer_semaphore;
    SemaphoreHandle_t touch_semaphore;
} yg_display_t;

static yg_display_t yg_display;

void yg_draw_rect(const esp_lcd_panel_io_handle_t io, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const void *color_data);
void yg_tick(void *arg);
bool yg_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
void yg_lcd_touch_callback(esp_lcd_touch_handle_t tp);

void yg_display_init(void) {
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << YG_PIN_NUM_LCD_BL,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "spi_bus_initialize");
    spi_bus_config_t bus_config = {
        .sclk_io_num = YG_PIN_NUM_LCD_SCLK,
        .mosi_io_num = YG_PIN_NUM_LCD_MOSI,
        .miso_io_num = YG_PIN_NUM_LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = YG_LCD_BUF_SIZE,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(YG_LCD_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "esp_lcd_new_panel_io_spi");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = YG_PIN_NUM_LCD_DC,
        .cs_gpio_num = YG_PIN_NUM_LCD_CS,
        .pclk_hz = YG_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = YG_LCD_CMD_BITS,
        .lcd_param_bits = YG_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    esp_lcd_panel_io_handle_t panel_io_handle;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)YG_LCD_SPI_HOST, &io_config, &panel_io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = YG_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    ESP_LOGI(TAG, "Install GC9A01 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io_handle, &panel_config, &yg_display.panel_handle));

    ESP_LOGI(TAG, "Initialize transfer semaphore and display buffers");
    vSemaphoreCreateBinary(yg_display.transfer_semaphore);
    yg_display.buf1 = heap_caps_malloc(YG_LCD_BUF_SIZE, MALLOC_CAP_DMA);
    assert(yg_display.buf1);
    yg_display.buf2 = heap_caps_malloc(YG_LCD_BUF_SIZE, MALLOC_CAP_DMA);
    assert(yg_display.buf2);

    ESP_LOGI(TAG, "Install callbacks");
    const esp_timer_create_args_t tick_timer_args = {
        .arg = &yg_display,
        .callback = &yg_tick,
        .name = "tick"
    };

    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, 16666));

    const esp_lcd_panel_io_callbacks_t io_callbacks = { .on_color_trans_done = yg_color_trans_done, };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(panel_io_handle, &io_callbacks, &yg_display));

    ESP_LOGI(TAG, "Initialize panel then turn on backlight");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(yg_display.panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(yg_display.panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(yg_display.panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(yg_display.panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(yg_display.panel_handle, true));
    ESP_ERROR_CHECK(gpio_set_level(YG_PIN_NUM_LCD_BL, YG_LCD_BK_LIGHT_ON_LEVEL));

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
    
    ESP_LOGI(TAG, "esp_lcd_new_panel_io_i2c");
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = 400 * 1000;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle , &tp_io_config, &tp_io_handle));

    ESP_LOGI(TAG, "esp_lcd_touch_new_i2c_cst816s");
    yg_display.touch_semaphore = xSemaphoreCreateBinary();
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = YG_LCD_H_RES,
        .y_max = YG_LCD_V_RES,
        .rst_gpio_num = YG_PIN_NUM_TP_RST,
        .int_gpio_num = YG_PIN_NUM_TP_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = yg_lcd_touch_callback,
        .user_data = &yg_display,
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &yg_display.touch_handle));

    // TODO: Refactor this out to a main loop in the main.c file
    while(1) {
        if (xSemaphoreTake(yg_display.touch_semaphore, 0) == pdTRUE) {
            esp_lcd_touch_read_data(yg_display.touch_handle); // read only when ISR was triggled
            uint16_t touch_x[1];
            uint16_t touch_y[1];
            uint16_t touch_strength[1];
            uint8_t touch_cnt = 0;

            bool touchpad_pressed = esp_lcd_touch_get_coordinates(yg_display.touch_handle, touch_x, touch_y, touch_strength, &touch_cnt, 1);
            if (touchpad_pressed && touch_cnt > 0) {
                ESP_LOGI(TAG, "Touchpad pressed at x: %d, y: %d, strength: %d", touch_x[0], touch_y[0], touch_strength[0]);
            }
        }
        // sleep
        vTaskDelay(1);
    }
}

const uint16_t colors[8] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF, 0x0000};

void yg_tick(void *arg) {
    yg_display_t *yg_display_p = (yg_display_t *)arg;
    uint16_t tick_counter = yg_display_p->tick_num;

    uint16_t *buf;
    if (tick_counter % 2 == 0) {
        buf = yg_display_p->buf1;
    } else {
        buf = yg_display_p->buf2;
    }

    // TODO: Do some useful drawing instead
    for (uint32_t i = 0; i < YG_LCD_BUF_PIXEL_COUNT; i ++) {
        buf[i] = colors[tick_counter];
    }

    xSemaphoreTake(yg_display_p->transfer_semaphore, portMAX_DELAY);
    esp_lcd_panel_draw_bitmap(yg_display_p->panel_handle,
        0, tick_counter * (YG_LCD_V_RES/YG_LCD_FILL_BLOCKS),
        YG_LCD_H_RES, (tick_counter + 1) * (YG_LCD_V_RES/YG_LCD_FILL_BLOCKS),
        buf);
    yg_display_p->tick_num = (tick_counter + 1) % YG_LCD_FILL_BLOCKS;
}

bool yg_color_trans_done(esp_lcd_panel_io_handle_t panel_io_handle, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    BaseType_t woken = false;
    yg_display_t *yg_display_p = (yg_display_t *)user_ctx;
    xSemaphoreGiveFromISR(yg_display_p->transfer_semaphore, &woken);
    return woken;
}

void yg_lcd_touch_callback(esp_lcd_touch_handle_t tp) {
    BaseType_t woken = false;
    yg_display_t *yg_display_p = (yg_display_t *)tp->config.user_data;
    xSemaphoreGiveFromISR(yg_display_p->touch_semaphore, &woken);
    if (woken) { portYIELD_FROM_ISR(); }
}