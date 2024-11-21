set(SOURCES main.c yg_display.c)
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS "."
                    REQUIRES esp_timer nvs_flash)