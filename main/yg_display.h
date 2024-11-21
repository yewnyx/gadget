#pragma once

#include <stdbool.h>

#include "esp_lcd_touch.h"

void yg_display_init(void);
bool yg_display_get_touches(esp_lcd_touch_handle_t *touch);