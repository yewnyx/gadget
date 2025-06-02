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

#include "driver/i2c_types.h"

#include "yg_config.h"

static const char *TAG = "yg_display";

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
static char charbuf[YG_LCD_LINE_COUNT][YG_LCD_LINE_WIDTH] = { 0 };
//static char charbuf[YG_LCD_LINE_COUNT * YG_LCD_LINE_WIDTH];
static char chars[95] = { ' ', '!', '"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '[', '\\', ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~' };

void yg_draw_rect(const esp_lcd_panel_io_handle_t io, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, const void *color_data);
void yg_tick(void *arg);
bool yg_color_trans_done(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
void yg_lcd_touch_callback(esp_lcd_touch_handle_t tp);

void yg_display_init(i2c_master_bus_handle_t i2c_bus_handle) {
    ESP_LOGI(TAG, "Fill charbuf with characters");
    int char_index = 0;
    for (int i = 0; i < YG_LCD_LINE_COUNT; i++) {
        for (int j = 0; j < YG_LCD_LINE_WIDTH; j++) {
            int index = char_index % 95;
            charbuf[i][j] = chars[index];
            char_index++;
        }
    }

    // int test_char_index = 0;
    // for (int line_num = 0; line_num < YG_LCD_LINE_COUNT; line_num++) {
    //     for (int char_index = 0; char_index < YG_LCD_LINE_WIDTH; char_index++) {
    //         int index = test_char_index % 95;
    //         charbuf[line_num * YG_LCD_LINE_WIDTH + char_index] = chars[index];
    //         test_char_index++;
    //     }
    // }

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

    ESP_LOGI(TAG, "esp_lcd_new_panel_io_i2c");
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_io_config.scl_speed_hz = 400 * 1000;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_handle , &tp_io_config, &tp_io_handle));

    ESP_LOGI(TAG, "Install CST816S touch panel driver");
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
    yg_display.buf1 = heap_caps_malloc(YG_LCD_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    assert(yg_display.buf1);
    yg_display.buf2 = heap_caps_malloc(YG_LCD_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    assert(yg_display.buf2);

    ESP_LOGI(TAG, "Start display tick timer");
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

    ESP_LOGI(TAG, "Reset and init LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(yg_display.panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(yg_display.panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(yg_display.panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(yg_display.panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(yg_display.panel_handle, true));
}

bool yg_display_get_touches(esp_lcd_touch_handle_t *touch) {
    if (xSemaphoreTake(yg_display.touch_semaphore, 0) == pdTRUE) {
        esp_lcd_touch_read_data(yg_display.touch_handle);
        if (touch) { *touch = yg_display.touch_handle; }
        return true;
    } else {
        if (touch) { *touch = NULL; }
        return false;
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

    const int BLOCK_PX_HEIGHT = (YG_LCD_V_RES / YG_LCD_FILL_BLOCKS);
    const int LINES_PER_BLOCK = (BLOCK_PX_HEIGHT / FONT_HEIGHT);
    int line_start_index = tick_counter * LINES_PER_BLOCK;

    for (int line_num = line_start_index; line_num < line_start_index + LINES_PER_BLOCK; line_num++) {
        for (int char_num = 0; char_num < YG_LCD_LINE_WIDTH; char_num++) {
            int char_to_draw = charbuf[line_num][char_num];
            if (char_to_draw < 32 || char_to_draw >= 127) {
                // Skip invalid characters
                ESP_LOGE(TAG, "Invalid character: %d at position (%d, %d)", char_to_draw, line_num, char_num);
                continue;
            }

            // if (char_to_draw != ' ') {
            //     ESP_LOGI(TAG, "Drawing character '%c' (ASCII %d) at position (%d, %d)", char_to_draw, char_to_draw, line_num, char_num);
            // }

            uint16_t *char_bits = spleen_6x12_chars[char_to_draw - 32];
            for (int x = 0; x < FONT_WIDTH; x++) {
                for (int y = 0; y < FONT_HEIGHT; y++) {
                    bool is_pixel_set = (char_bits[y] & (1 << (FONT_WIDTH - x + 1))) != 0;
                    //bool is_pixel_set = x % 3 == 0 || x % 3 == 1 || y % 5 == 0 || y % 5 == 1;

                    // Offset by line and line height, then by line width, then by pixel
                    int pixel_index = YG_LCD_H_RES * ((line_num % LINES_PER_BLOCK) * FONT_HEIGHT + y) + (FONT_WIDTH * char_num) + x;
                    // buf[pixel_index] = is_pixel_set ? 0xF800 : 0xF81F;
                    buf[pixel_index] = is_pixel_set ? colors[tick_counter % 7] : 0x0000;
                }
            }
        }
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