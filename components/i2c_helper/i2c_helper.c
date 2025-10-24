#include "i2c_helper.h"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#define I2C_SN65DSI83_ADDR  0x2C
#define I2C_MASTER_SCL_IO   49//8      // <-- change for own hardware
#define I2C_MASTER_SDA_IO   50//7
#define I2C_MASTER_FREQ_HZ  100000 // KHz
#define I2C_PORT            I2C_NUM_0 

#define TAG                 "i2c_helper"

#define I2C_MASTER_TIMEOUT_MS (1000)


/**
 * @brief Read a sequence of bytes SN65DSI registers
 */
static esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a SN65DSI register
 */
static esp_err_t register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_SN65DSI83_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static void dump_registers(i2c_master_dev_handle_t dev_handle, uint8_t start, uint8_t end)
{
    uint8_t val;
    esp_err_t ret;

    printf("Dumping device registers 0x%02X-0x%02X:\n", start, end);
    printf("    00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n\n");

    for (uint8_t row = start & 0xF0; row <= end; row += 0x10) {

        printf("%02X: ", row);

        for (uint8_t col = 0; col < 16; col++) {
            uint8_t reg = row + col;

            if (reg < start || reg > end) {
                printf("   "); // outside requested range
                continue;
            }

            ret = register_read(dev_handle, reg, &val, 1);

            if (ret == ESP_OK) {
                printf("%02X ", val);
            } else {
                // if NACK or any other error → mark as "--"
                printf("-- ");
            }
        }
        printf("\n");
        if (row == 0xF0) break;
    }
}

// Configure SN65DSI83 test pattern registers
// Example timing: 800x600 @ 60 Hz (adjust values for your panel)
static esp_err_t sn65dsi83_enable_test_pattern(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    // LVDS regs
    ret = register_write_byte(dev_handle, 0x18, 0x78);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x19, 0x00);   // High 4 bits
    if (ret != ESP_OK) return ret;
    
    ret = register_write_byte(dev_handle, 0x1A, 0x03);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x1B, 0x00);   // High 4 bits
    if (ret != ESP_OK) return ret;

    // Active line length (horizontal resolution) = 1280 (0x320)
    ret = register_write_byte(dev_handle, 0x20, 0x00);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x21, 0x05);   // High 4 bits
    if (ret != ESP_OK) return ret;
    // NEED 0 fill 0 
    ret = register_write_byte(dev_handle, 0x22, 0x00);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x23, 0x00);   // High 4 bits
    if (ret != ESP_OK) return ret;

    // Vertical display size = 600 (0x320)
    ret = register_write_byte(dev_handle, 0x24, 0x20);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x25, 0x03);   // High 4 bits
    if (ret != ESP_OK) return ret;
    // NEED 0 fill 0 
    ret = register_write_byte(dev_handle, 0x26, 0x00);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x27, 0x00);   // High 4 bits
    if (ret != ESP_OK) return ret;

    // Sync delay = 40 pixels (must be >= 32)
    ret = register_write_byte(dev_handle, 0x28, 0x20);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x29, 0x00);   // High 4 bits
    if (ret != ESP_OK) return ret;

    // NEED 0 fill 0
    ret = register_write_byte(dev_handle, 0x2A, 0x00);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x2B, 0x00);   // High 4 bits
    if (ret != ESP_OK) return ret;

    // HSync pulse width = 128 pixels
    ret = register_write_byte(dev_handle, 0x2C, 0x01);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x2D, 0x00);   // High 2 bits
    if (ret != ESP_OK) return ret;

    // VSync pulse width = 4 lines
    ret = register_write_byte(dev_handle, 0x30, 0x01);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x31, 0x00);   // High 2 bits
    if (ret != ESP_OK) return ret;

    // Horizontal back porch = 88 pixels
    ret = register_write_byte(dev_handle, 0x34, 0x04);
    if (ret != ESP_OK) return ret;

    // Vertical back porch = 23 lines
    ret = register_write_byte(dev_handle, 0x36, 0x01);
    if (ret != ESP_OK) return ret;

    // Horizontal front porch = 40 pixels
    ret = register_write_byte(dev_handle, 0x38, 0x40);
    if (ret != ESP_OK) return ret;

    // Vertical front porch = 1 line
    ret = register_write_byte(dev_handle, 0x3A, 0x28);
    if (ret != ESP_OK) return ret;

    // Enable test pattern generation (bit 4 at 0x3C)
    ret = register_write_byte(dev_handle, 0x3C, 0x10);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

esp_err_t write_all_regs_zero(i2c_master_dev_handle_t dev_handle,
                              uint8_t start, uint8_t end)
{
    esp_err_t ret;
    ESP_LOGI("REG_TO_ZERO", "FILL REGS 0x%02X-0x%02X:\n", start, end);
    for (uint16_t addr = start; addr <= end; addr++) {
        ret = register_write_byte(dev_handle, (uint8_t)addr, 0x00);
        if (ret != ESP_OK) {
            ESP_LOGE("REG_ZERO", "Failed to write reg 0x%02X (err=0x%x)",
                    addr, ret);
            return ret;
        }
    }
    return ESP_OK;
}


void run_i2c()
{
    //uint8_t data[2];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");
    write_all_regs_zero(dev_handle, 0x00, 0xFF);
    // Dump all posibly register 0x09-0xE5
    dump_registers(dev_handle, 0x00, 0xFF);

    esp_err_t ret;

    ret = sn65dsi83_enable_test_pattern(dev_handle);
    if (ret != ESP_OK) {
        printf("I2C write failed, err=0x%x\n", ret);
    }

    //Again dump after TEST PATTERN
    dump_registers(dev_handle, 0x00, 0xFF);

    /* Read the SN65DSI83's PLL register, it should have the value 0x83(locked) */
    //ESP_ERROR_CHECK(register_read(dev_handle, 0x0a, data, 1));
    //ESP_LOGI(TAG, "PLL is locked = %X", data[0]); 


}
