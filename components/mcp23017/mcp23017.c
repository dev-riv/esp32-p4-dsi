#include "mcp23017.h"
#include "driver/i2c_master.h"   // uses ESP-IDF's example helper header (wraps standard i2c_master functions)
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "mcp23017";
static i2c_port_t s_i2c_num = I2C_NUM_0;
static uint8_t s_i2c_addr = 0x21;
static i2c_master_bus_handle_t s_i2c_bus_handle = NULL;
static i2c_master_dev_handle_t s_i2c_dev_handle = NULL;
static uint8_t s_sda_io_num = 7;
static uint8_t s_scl_io_num = 6;
static uint32_t s_i2c_freq = 100000;
static uint32_t s_i2c_timeout = 1000;

/* MCP23017 register map (A-bank) */
#define MCP23017_IODIRA   0x00
#define MCP23017_IODIRB   0x01
#define MCP23017_GPIOA    0x12
#define MCP23017_GPIOB    0x13
#define MCP23017_OLATA    0x14
#define MCP23017_OLATB    0x15

#define GPA0 (1 << 0)   // 0b00000001
#define GPA1 (1 << 1)   // 0b00000010
#define GPA2 (1 << 2)   // 0b00000100
#define GPA3 (1 << 3)   // 0b00001000
#define GPA4 (1 << 4)   // 0b00010000
#define GPA5 (1 << 5)   // 0b00100000
#define GPA6 (1 << 6)   // 0b01000000
#define GPA7 (1 << 7)   // 0b10000000


/**
  * @brief Read a sequence of bytes registers
  */
 static esp_err_t register_read(uint8_t reg_addr, uint8_t *data, size_t len)
 {
     return i2c_master_transmit_receive(s_i2c_dev_handle, &reg_addr, 1, data, len,
      s_i2c_timeout / portTICK_PERIOD_MS);
 }
 
 /**
  * @brief Write a byte to a register
  */
 static esp_err_t register_write_byte(uint8_t reg_addr, uint8_t data)
 {
     uint8_t write_buf[2] = {reg_addr, data};
     return i2c_master_transmit(s_i2c_dev_handle, write_buf, sizeof(write_buf),
        s_i2c_timeout / portTICK_PERIOD_MS);
 }

 /**
  * @brief Read pin state (true = high, false = low)
  */
 esp_err_t mcp23017_read_pin(uint8_t pin, bool *level)
{
    if (!level) return ESP_ERR_INVALID_ARG;

    uint8_t reg = (pin < 8) ? MCP23017_GPIOA : MCP23017_GPIOB;
    uint8_t bit = pin % 8;

    uint8_t val;
    ESP_RETURN_ON_ERROR(register_read(reg, &val, 1), TAG,
                                                            "read gpio fail");

    *level = (val >> bit) & 0x01;
    return ESP_OK;
}

 /**
  * @brief Write pin high/low
  */
esp_err_t mcp23017_write_pin(uint8_t pin, bool level)
{
    uint8_t reg = (pin < 8) ? MCP23017_GPIOA : MCP23017_GPIOB;
    uint8_t bit = pin % 8;

    uint8_t val;
    ESP_RETURN_ON_ERROR(register_read(reg, &val, 1), TAG,
            "read gpio fail");
    //ESP_LOGI(TAG, "READ VALUE: %u, At reg: %u, bit: %u", val, reg, bit);
    if (level)
        val |= (1 << bit);
    else
        val &= ~(1 << bit);

    ESP_RETURN_ON_ERROR(register_write_byte(reg, val), TAG,
            "write gpio fail");
    return ESP_OK;
}
 
static void i2c_master_init()
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = s_i2c_num,
        .sda_io_num = s_sda_io_num,
        .scl_io_num = s_scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &s_i2c_bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_i2c_addr,
        .scl_speed_hz = s_i2c_freq,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus_handle,
                                        &dev_config, &s_i2c_dev_handle));
}


esp_err_t mcp23017_init()
{

    esp_err_t ret;

    // init i2c master using i2c_master.h helper (this file is a small wrapper example included with esp-idf samples)
    i2c_master_init();
   
    // set GPA (A) direction to outputs (0 = output)
    ret = register_write_byte(MCP23017_IODIRA, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "writing IODIRA failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(20);

    // set GPA (B) direction to outputs (0 = output)
    ret = register_write_byte(MCP23017_IODIRB, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "writing IODIRB failed: %s", esp_err_to_name(ret));
        return ret;
    }

    //// set OLATA all high (CS inactive = HIGH)
    //ret = register_write_byte(MCP23017_OLATA, 0xFF);
    //if (ret != ESP_OK) {
    //    ESP_LOGE(TAG, "writing OLATA failed: %s", esp_err_to_name(ret));
    //    return ret;
    //}

    ESP_LOGI(TAG, "MCP23017 initialized at 0x%02X", s_i2c_addr);
    return ESP_OK;
}
