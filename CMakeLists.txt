idf_component_register(
    SRCS "mcp23017.c" "sn65_conf.c" "esp_lcd_sn65dsi.c" 
    INCLUDE_DIRS "include"
    PRIV_REQUIRES "driver" "esp_timer" 
    REQUIRES "esp_lcd"
    PRIV_INCLUDE_DIRS "$ENV{IDF_PATH}/components/esp_lcd/dsi"
    )

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
