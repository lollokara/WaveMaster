idf_component_register(
    SRCS "main.c" "zb_driver.c" "zb_endstop.c" "zb_motion.c" "zb_settings.c" "zb_cal.c" "zb_cal_search.c"
         "zb_net.c" "zb_web.c" "../../shared/zbro_proto.c"
    INCLUDE_DIRS "." "../../shared"
    REQUIRES driver esp_driver_gpio esp_driver_rmt esp_timer nvs_flash esp_wifi esp_netif esp_event
             esp_http_server json lwip esp_hw_support
)
