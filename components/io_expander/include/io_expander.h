#pragma once
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

/**
 * @brief Initialize I²C master bus and attach a device
 *
 * This function configures the ESP32 as an I²C master, creates a bus handle,
 * and attaches a single I²C device (such as the MCP23017 IO expander) to it.
 *
 * It sets up the bus using predefined configuration macros:
 *   - `I2C_PORT`              → I²C controller number (e.g., I2C_NUM_0)
 *   - `I2C_MASTER_SDA_IO`     → SDA pin number
 *   - `I2C_MASTER_SCL_IO`     → SCL pin number
 *   - `I2C_MASTER_FREQ_HZ`    → I²C clock frequency
 *   - `I2C_IOE_ADDR`          → Target device 7-bit address
 *
 * The function returns the created bus and device handles through
 * the provided pointers.
 *
 * @param[out] bus_handle Pointer to store the created I²C master bus handle
 * @param[out] dev_handle Pointer to store the created I²C device handle
 *
 * @note Both handles must be valid when using `i2c_master_transmit()` or
 *       `i2c_master_transmit_receive()` functions.
 *
 * Example:
 * @code
 * i2c_master_bus_handle_t bus;
 * i2c_master_dev_handle_t dev;
 * i2c_master_init(&bus, &dev);
 *
 * // Now dev can be used with MCP23017 or other I²C devices
 * mcp23017_set_pin_dir(dev, 7, true);
 * mcp23017_write_pin(dev, 7, true);
 * @endcode
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);


// Initialize I2C bus and run to dump registers
void io_expander_run();

// Set the gpio path to use
esp_err_t mcp23017_set_pin_dir(i2c_master_dev_handle_t dev, uint8_t pin, bool is_output);
/*
 * @param[in] dev   I²C device handle (address = 0x20–0x27)
 * @param[in] pin   Pin number (0–15)
 * @param[in] level Logic level: true = HIGH, false = LOW
 *
 * @return ESP_OK on success, or error code on I²C failure
*/
esp_err_t mcp23017_write_pin(i2c_master_dev_handle_t dev, uint8_t pin, bool level);

// Read from the gpio
esp_err_t mcp23017_read_pin(i2c_master_dev_handle_t dev, uint8_t pin, bool *level);
