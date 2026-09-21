idf_component_register(
    SRCS "main.c" "ad3552r_board.c" "spi_probe.c" "laser_ctrl.c" "engrave_gpio.c"
         "prr_pwm.c" "dac_task.c" "grbl_task.c" "calib.c" "atmega_link.c"
         "laser_fill.c" "motion_planner.c"
    INCLUDE_DIRS "."
    REQUIRES ad3552r driver nvs_flash esp_timer
)
