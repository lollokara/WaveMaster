idf_component_register(
    SRCS "ad3552r.c" "no_os_spi_esp32.c" "no_os_gpio_esp32.c" "no_os_crc8.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)
