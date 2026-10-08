idf_component_register(
    SRCS "main.c" "host_serial.c" "grbl.c" "gcode_parse.c" "calib.c"
         "motion.c" "motion_task.c" "galvo_out.c" "dac_task.c"
         "laser_io.c" "ad3552r_board.c" "atmega_link.c"
    INCLUDE_DIRS "."
    REQUIRES ad3552r driver nvs_flash esp_timer esp_driver_usb_serial_jtag esp_driver_gptimer esp_driver_ledc esp_driver_spi esp_driver_uart esp_driver_gpio
)
