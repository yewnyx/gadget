#pragma once

#include <stdbool.h>

#include "esp_lcd_touch.h"
#include "driver/i2c_master.h"

void yg_display_init(i2c_master_bus_handle_t i2c_bus_handle);
bool yg_display_get_touches(esp_lcd_touch_handle_t *touch);