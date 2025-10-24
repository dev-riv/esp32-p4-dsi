#include "io_expander.h"


#define I2C_IOE_ADDR        0x21
#define I2C_MASTER_SCL_IO   6      // <-- change for own hardware
#define I2C_MASTER_SDA_IO   7
#define I2C_MASTER_FREQ_HZ  100000 // KHz
#define I2C_PORT            I2C_NUM_0 

#define TAG                 "IO_Expander_0x21"

#define I2C_MASTER_TIMEOUT_MS (2000)

// MCP23017 Register Addresses
#define MCP23017_IODIRA 0x00
#define MCP23017_IODIRB 0x01
#define MCP23017_GPIOA  0x12
#define MCP23017_GPIOB  0x13

/**
 * @brief Read a sequence of bytes registers
 */
static esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a register
 */
static esp_err_t register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
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
        .device_address = I2C_IOE_ADDR,
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

/**
 * @brief Set pin direction (input/output)
 */
esp_err_t mcp23017_set_pin_dir(i2c_master_dev_handle_t dev, uint8_t pin, bool is_output)
{
    uint8_t reg = (pin < 8) ? MCP23017_IODIRA : MCP23017_IODIRB;
    uint8_t bit = pin % 8;

    uint8_t dir;
    ESP_RETURN_ON_ERROR(register_read(dev, reg, &dir, 1), TAG, "read dir fail");

    if (is_output)
        dir &= ~(1 << bit);
    else
        dir |= (1 << bit);

    ESP_RETURN_ON_ERROR(register_write_byte(dev, reg, dir), TAG, "write dir fail");
    return ESP_OK;
}

/**
 * @brief Write pin high/low
 */
esp_err_t mcp23017_write_pin(i2c_master_dev_handle_t dev, uint8_t pin, bool level)
{
    uint8_t reg = (pin < 8) ? MCP23017_GPIOA : MCP23017_GPIOB;
    uint8_t bit = pin % 8;

    uint8_t val;
    ESP_RETURN_ON_ERROR(register_read(dev, reg, &val, 1), TAG, "read gpio fail");

    if (level)
        val |= (1 << bit);
    else
        val &= ~(1 << bit);

    ESP_RETURN_ON_ERROR(register_write_byte(dev, reg, val), TAG, "write gpio fail");
    return ESP_OK;
}

/**
 * @brief Read pin state (true = high, false = low)
 */
esp_err_t mcp23017_read_pin(i2c_master_dev_handle_t dev, uint8_t pin, bool *level)
{
    if (!level) return ESP_ERR_INVALID_ARG;

    uint8_t reg = (pin < 8) ? MCP23017_GPIOA : MCP23017_GPIOB;
    uint8_t bit = pin % 8;

    uint8_t val;
    ESP_RETURN_ON_ERROR(register_read(dev, reg, &val, 1), TAG, "read gpio fail");

    *level = (val >> bit) & 0x01;
    return ESP_OK;
}


void io_expander_run()
{
    //uint8_t data[2];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C for 0x21 initialized successfully");
    // Dump all posibly register 0x09-0xE5
    dump_registers(dev_handle, 0x00, 0xFF);
}
