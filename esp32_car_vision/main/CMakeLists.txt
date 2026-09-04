idf_component_register(
    SRCS "car_vision_main.c" "car_display.c" "tjpgd.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_spi esp_timer
)
