#ifndef MCP23017_H
#define MCP23017_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the MCP23017 I/O expander
 *
 * @param i2c_num   I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param sda_gpio  GPIO number for SDA pin
 * @param scl_gpio  GPIO number for SCL pin
 * @param i2c_addr  I2C address (0x20–0x27 depending on A0/A1/A2 pins)
 *
 * @return ESP_OK on success, otherwise appropriate esp_err_t
 */
esp_err_t mcp23017_init();

esp_err_t mcp23017_read_pin(uint8_t pin, bool *level);

esp_err_t mcp23017_write_pin(uint8_t pin, bool level);

#ifdef __cplusplus
}
#endif

#endif /* MCP23017_H */

