idf_component_register(
    SRCS "sn65_conf.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_lcd driver mcp23017
    PRIV_REQUIRES esp_timer
)

