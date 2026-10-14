# ESP-IDF component registration for the ESPFleet agent.
#
# Drop this directory into your project's components/ folder (or add it with
# EXTRA_COMPONENT_DIRS) and the agent builds as a normal IDF component:
#
#   set(EXTRA_COMPONENT_DIRS ".../ESPFleet/firmware/espfleet")
#
# PlatformIO ignores this file and uses library.json instead.

idf_component_register(
    SRCS "src/EspFleet.cpp" "src/espfleet_c.cpp"
    INCLUDE_DIRS "src"
    REQUIRES
        esp_http_client
        app_update
        esp_wifi
        esp_netif
        esp_timer
        esp_partition
        esp_app_format
        esp_hw_support
        esp_system
        spi_flash
        nvs_flash
        mbedtls
        lwip
)

# The mDNS component is optional; EspFleet.cpp guards on __has_include("mdns.h").
idf_build_get_property(build_components BUILD_COMPONENTS)
if("espressif__mdns" IN_LIST build_components OR "mdns" IN_LIST build_components)
    idf_component_get_property(mdns_lib espressif__mdns COMPONENT_LIB)
    if(NOT mdns_lib)
        idf_component_get_property(mdns_lib mdns COMPONENT_LIB)
    endif()
    if(mdns_lib)
        target_link_libraries(${COMPONENT_LIB} PRIVATE ${mdns_lib})
        message(STATUS "espfleet: mDNS component found, _espfleet._tcp advertising enabled")
    endif()
else()
    message(STATUS "espfleet: no mDNS component, advertising disabled "
                   "(run: idf.py add-dependency \"espressif/mdns\")")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/cmake/espfleet_git.cmake")
espfleet_inject_git_info(${COMPONENT_LIB})
