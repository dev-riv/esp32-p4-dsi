#include "sn65_conf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "mcp23017.h"

#define I2C_SN65DSI83_ADDR  0x2C
#define I2C_MASTER_SCL_IO   49//8      // <-- change for own hardware
#define I2C_MASTER_SDA_IO   50//7
#define I2C_MASTER_FREQ_HZ  100000 // KHz
#define I2C_PORT            I2C_NUM_1 

#define TAG                 "SN65_CONFIG"

#define I2C_MASTER_TIMEOUT_MS (1000)

static i2c_master_dev_handle_t s_dev_handle = NULL;
static i2c_master_bus_handle_t s_bus_handle = NULL;

static void dump_registers(i2c_master_dev_handle_t dev_handle, uint8_t start, uint8_t end);

//EN control using mcp23017
static inline void en_assert(uint8_t gpa_pin, bool en)
{
    mcp23017_write_pin(gpa_pin, en);
    vTaskDelay(pdMS_TO_TICKS(10));
}

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
static void i2c_master_init()
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &s_bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_SN65DSI83_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus_handle, &dev_config, &s_dev_handle));
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

void sn_pll_en(bool en)
{
    esp_err_t ret;

    if(en) {
        ret = register_write_byte(s_dev_handle, 0x0d, 0x01); // Pll enable
        ESP_LOGI(TAG, "Set PLL ENABLE");
    } else {
        ret = register_write_byte(s_dev_handle, 0x0d, 0x00); // Pll disable
        ESP_LOGI(TAG, "Set PLL DISABLE");
    }
    if (ret != ESP_OK) ESP_LOGE(TAG, "PLL set error %s", ret);
}

void sn_chip_en(bool en)
{
    //mcp23x17_set_level(&mcp_dev, 11, en);
    en_assert(11, en);
    ESP_LOGI(TAG, "Set SN65 EN pin %d", en);
}

void sn_soft_reset(bool set)
{
    if(set) {
        ESP_LOGI(TAG, "Perform soft reset");
        esp_err_t ret = register_write_byte(s_dev_handle, REG_RC_RESET, BIT(0));
        if (ret != ESP_OK) ESP_LOGE(TAG, "Soft Reset enable error %s", ret);
    } else {
        esp_err_t ret = register_write_byte(s_dev_handle, REG_RC_RESET, 0x00);
        if (ret != ESP_OK) ESP_LOGE(TAG, "Soft Reset  disable error %s", ret);
    } 
}

bool is_sn_pll_locked()
{
    uint8_t val;
    esp_err_t ret;

    ret = register_read(s_dev_handle, 0x0a, &val, 1);

    if(val & BIT(7)) {
        return true;
    }
    if (ret != ESP_OK) ESP_LOGE(TAG, "GET PLL STATUS ERROR %s", ret);
    return false;
}

void pll_checker_task(void *arg)
{
    bool lock = is_sn_pll_locked();
    ESP_LOGI(TAG, "PLL LOCKED %d", lock);
    vTaskDelay(pdMS_TO_TICKS(10));  
}

esp_err_t clear_error_reg()
{
    uint8_t pval;
    esp_err_t ret;

    ret = register_read(s_dev_handle, REG_IRQ_STAT, &pval, 1);
    ret = register_write_byte(s_dev_handle, REG_IRQ_STAT, pval);
    vTaskDelay(pdMS_TO_TICKS(10));  
    ret = register_read(s_dev_handle, REG_IRQ_STAT, &pval, 1);
    if (pval) {
        ESP_LOGW(TAG, "Unexpected link status 0x%02x\n", pval); 
    } 
    return ret;
}

void show_regs()
{
    dump_registers(s_dev_handle, 0x00, 0xFF);
}

//static esp_err_t additional_conf(i2c_master_dev_handle_t dev_handle)
//{
//    esp_err_t ret;
//    ret = register_write_byte(dev_handle, 0x0e, 0x01);// * 
//    if (ret != ESP_OK) return ret;
//    ret = register_write_byte(dev_handle, 0x0f, 0x01);// * 
//    if (ret != ESP_OK) return ret;
//
//    ret = register_write_byte(dev_handle, 0x13, 0x5a);// *
//    if (ret != ESP_OK) return ret;
//    ret = register_write_byte(dev_handle, 0x14, 0x5a);// *
//    if (ret != ESP_OK) return ret;
//    ret = register_write_byte(dev_handle, 0x15, 0x5a);// *
//    if (ret != ESP_OK) return ret;
//    ret = register_write_byte(dev_handle, 0x16, 0x5a);// *
//    if (ret != ESP_OK) return ret;
//    ret = register_write_byte(dev_handle, 0x17, 0x5a);// *
//    if (ret != ESP_OK) return ret;
//
//    return ESP_OK;
//}

// Configure SN65DSI83 test pattern registers
// Example timing: 1280x800 @ 60 Hz (adjust values for your panel)
static esp_err_t sn65dsi83_enable_test_pattern(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;

    //ret = additional_conf(s_dev_handle);
    //if (ret != ESP_OK) return ret;
    // PLL to 0 to correct config 0A & 0B
    ret = register_write_byte(dev_handle, 0x0d, 0x00);// Pll en 01
    if (ret != ESP_OK) return ret;

    //0x03(37.5-62.5) | 0x04(62.5-87.5) | 0x05(87.5-112.5) | 0x06(112.5-137.5) 
    ret = register_write_byte(dev_handle, 0x0a, 0x03);// LVDS expected range
    if (ret != ESP_OK) return ret;

    // 0x20(div by 5) 0x28(div by 6) 0x30(div by 7)
    ret = register_write_byte(dev_handle, 0x0b, 0x28);// DSI clock divider 
    if (ret != ESP_OK) return ret;

    //ret = register_write_byte(dev_handle, 0x0d, 0x01);// Pll en 01
    //if (ret != ESP_OK) return ret;
    // Choosen 2 DSI lanes
    ret = register_write_byte(dev_handle, 0x10, 0x30);// Set DSI lanes
    if (ret != ESP_OK) return ret;
    // 0x59(445>=450) 0x5a(450>=455)
    ret = register_write_byte(dev_handle, 0x12, 0x5a);// Set DSI input clock
    if (ret != ESP_OK) return ret;
    // def 70,, rpi5 18
    ret = register_write_byte(dev_handle, 0x18, 0x18);// BPP_MODE 24 or 18 
    if (ret != ESP_OK) return ret;

    // Active line length (horizontal resolution) = 1280 (0x500) addr 0x20-21
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW, 0x00);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH, 0x05);   // High 4 bits
    if (ret != ESP_OK) return ret;

    // Vertical display size = 800 (0x320) addr 0x24-25
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW, 0x20);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH, 0x03);   // High 4 bits
    if (ret != ESP_OK) return ret;

    // HSync pulse width = 20 pixels | addr 0x2c-2d
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW, 0x14);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH, 0x00);   // High 2 bits
    if (ret != ESP_OK) return ret;

    ret = register_write_byte(dev_handle,
        REG_VID_CHA_SYNC_DELAY_LOW, 0x40);   // Low 8 bits
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_SYNC_DELAY_HIGH, 0x00);   // Low 3 bits

    // VSync pulse width = 10 lines | addr 0x30-31
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW, 0x0a);   // Low 8 bits
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH, 0x00);   // High 2 bits
    if (ret != ESP_OK) return ret;

    // Horizontal back porch = 68 (88-20) pixels | addr 0x34
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_HORIZONTAL_BACK_PORCH, 0x44);
    if (ret != ESP_OK) return ret;

    // Vertical back porch = 13 (23-10) lines | addr 0x36
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_VERTICAL_BACK_PORCH , 0x0d);
    if (ret != ESP_OK) return ret;

    // Horizontal front porch = 72 pixels | addr 0x38
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_HORIZONTAL_FRONT_PORCH, 0x48);
    if (ret != ESP_OK) return ret;

    // Vertical front porch = 15 line | addr 0x3a
    ret = register_write_byte(dev_handle,
        REG_VID_CHA_VERTICAL_FRONT_PORCH, 0x0f);
    if (ret != ESP_OK) return ret;

    // Enable test pattern generation (bit 4 at 0x3C)
    ret = register_write_byte(dev_handle, 0x3C, 0x00);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

static esp_err_t write_all_regs_zero(i2c_master_dev_handle_t dev_handle,
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

void sn65dsi_fill_regs()
{
    //write_all_regs_zero(dev_handle, 0x00, 0xFF);
    // Dump all posibly register 0x09-0xE5
    //dump_registers(s_dev_handle, 0x00, 0xFF);

    esp_err_t ret;

    ret = sn65dsi83_enable_test_pattern(s_dev_handle);

    if (ret != ESP_OK) {
        printf("I2C write failed, err=0x%x\n", ret);
    } else {
        ESP_LOGI(TAG, "SN configured successfully (wait for SOFT RESET)");
    }

    dump_registers(s_dev_handle, 0x00, 0xFF);
}

//static void mcp23017_init()
//{
//    ESP_ERROR_CHECK(i2cdev_init());
//
//    uint8_t mcp_i2c_addr = 0x21;
//    uint8_t mcp_i2c_sda_num = 7;
//    uint8_t mcp_i2c_scl_num = 6;
//    uint8_t mcp_i2c_port = 0;
//
//    ESP_ERROR_CHECK(mcp23x17_init_desc(&mcp_dev, mcp_i2c_addr, mcp_i2c_port,
//        mcp_i2c_sda_num, mcp_i2c_scl_num));
//
//    //mcp23x17_set_mode(&mcp_dev, pin_num, MCP23X17_GPIO_INPUT);
//    // for buzzer
//    mcp23x17_set_mode(&mcp_dev, 5, MCP23X17_GPIO_OUTPUT);
//    vTaskDelay(pdMS_TO_TICKS(50));
//    // for display brightness
//    mcp23x17_set_mode(&mcp_dev, 7, MCP23X17_GPIO_OUTPUT);
//    vTaskDelay(pdMS_TO_TICKS(50));
//    // for LCD reset
//    mcp23x17_set_mode(&mcp_dev, 9, MCP23X17_GPIO_OUTPUT);
//    vTaskDelay(pdMS_TO_TICKS(50));
//    // for LCD standby control (3.3v ON with HIGH)
//    mcp23x17_set_mode(&mcp_dev, 10, MCP23X17_GPIO_OUTPUT);
//    vTaskDelay(pdMS_TO_TICKS(50));
//    // for sn65 enable to work
//    mcp23x17_set_mode(&mcp_dev, 11, MCP23X17_GPIO_OUTPUT);
//
//    vTaskDelay(pdMS_TO_TICKS(50));
//    mcp23x17_set_level(&mcp_dev, 7, 1); 
//    vTaskDelay(pdMS_TO_TICKS(50));
//    mcp23x17_set_level(&mcp_dev, 9, 1); 
//    vTaskDelay(pdMS_TO_TICKS(50));
//    mcp23x17_set_level(&mcp_dev, 10, 1); 
//    vTaskDelay(pdMS_TO_TICKS(50));
//    mcp23x17_set_level(&mcp_dev, 11, 1); 
//
//    // trigger buzzer as success config
//    for(int i = 0; i < 5; i++) {
//
//        mcp23x17_set_level(&mcp_dev, 5, 1); 
//        vTaskDelay(pdMS_TO_TICKS(80));
//        mcp23x17_set_level(&mcp_dev, 5, 0); 
//        vTaskDelay(pdMS_TO_TICKS(160));
//    }
//}



void sn65dsi_deinit()
{
    esp_err_t ret = ESP_OK;

    write_all_regs_zero(s_dev_handle, 0x00, 0xFF);

    ret = mcp23017_deinit();

    if (NULL != s_bus_handle){

        ret = i2c_master_bus_rm_device(s_dev_handle);
        ret = i2c_del_master_bus(s_bus_handle);
    }

    if (ESP_OK != ret){
        ESP_LOGW(TAG, "DEININT FAILED");
    }
}

void sn65dsi_init_and_enable()
{
    // Set up io expander 
    //mcp23017_init();
    // Init Expander channel
    mcp23017_init();

    //Set brightness high
    en_assert(7, 1);
    //Set SN enable to work( NOW set from outside)
    //en_assert(11, 1);
    //LCD standby control (turn 3.3v ON with HIGH)
    en_assert(10, 1);
    //LCD Reset
    en_assert(9, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    en_assert(9, 1);

    //for(int i = 0; i < 5; i++) {

    //    en_assert(5, 1);
    //    vTaskDelay(pdMS_TO_TICKS(3));
    //    en_assert(5, 0);
    //    vTaskDelay(pdMS_TO_TICKS(4));
    //}

    // configure I2C bus for interact with sn65dsi83 

    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized successfully");

}
