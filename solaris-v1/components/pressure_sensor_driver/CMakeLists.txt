idf_component_register(
    SRCS "bmp390.c"
    INCLUDE_DIRS "include"
    REQUIRES general esp_driver_spi
    PRIV_REQUIRES esp_driver_gpio
)
