idf_component_register(
    SRCS "mcp23017.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_i2c
)

