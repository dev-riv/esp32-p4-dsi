#include "sn65_conf.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "mcp23017.h"

#define I2C_SN65DSI83_ADDR  0x2C
#define SN65_EN_GPB3        11 // GPB3 8+3 -> GPA7(last pin gpa)+3+1=GPB3 (11)

#define I2C_MASTER_SCL_IO   49//8      // <-- change for own hardware
#define I2C_MASTER_SDA_IO   50//7
#define I2C_MASTER_FREQ_HZ  100000 // KHz
#define I2C_PORT            I2C_NUM_1 

#define TAG                 "SN65_CONFIG"

#define I2C_MASTER_TIMEOUT_MS (1000)

// Default configuration
static struct sn65dsi83_config default_config = {
    .dsi_lanes = 2,
    .dsi_channel_mode = 0, // DUAL
    .dsi_clk_divider = 2,
    .dsi_refclk_multiplier = 1,
    .lvds_link_config = 0, // AB link
    .lvds_bpp_mode = 0x3, // 24bpp mode for both channels
    .lvds_de_neg_polarity = false,
    .lvds_hs_neg_polarity = false,
    .lvds_vs_neg_polarity = false,
    .active_line_length = 1280,
    .vertical_display_size = 800,
    .sync_delay = 0,
    .hsync_pulse_width = 20,
    .vsync_pulse_width = 10,
    .h_back_porch = 88,
    .v_back_porch = 23,
    .h_front_porch = 72,
    .v_front_porch = 15
};

static sn65dsi83_ctx_t sn65_ctx = { .state = INIT_POWER_ON, .initialized = false };



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

// EN control using mcp23017
static inline void en_assert(uint8_t gpa_pin, bool en)
{
    mcp23017_write_pin(gpa_pin, en);
    vTaskDelay(10);
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
    //ADDitional start 
    ret = register_write_byte(dev_handle, 0x0a, 0x04);// clock range 62.5-87.5
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x0b, 0x01);// Ref Clock multy 

    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev_handle, 0x0d, 0x01);// Pll en 01
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

    //ret = register_write_byte(dev_handle,
    //    REG_VID_CHA_SYNC_DELAY_LOW, 0x20);   // Low 8 bits

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
    ret = register_write_byte(dev_handle, 0x3C, 0x10);
    if (ret != ESP_OK) return ret;


    return ESP_OK;
}

// Configure SN65DSI83 registers
static esp_err_t sn65dsi83_configure(i2c_master_dev_handle_t dev, sn65dsi83_config_t config)
{
    esp_err_t ret;
    // Configure DSI lanes
    uint8_t dsi_lane_val = ((config.dsi_lanes & 0x3) << 3) | // CHA lanes
                           ((config.dsi_lanes & 0x3) << 1);   // CHB lanes

    if (config.dsi_channel_mode == 0) {
        // Dual channel mode
        dsi_lane_val |= 0; // DSI_CHANNEL_MODE_DUAL
    } else {
        // Single channel mode
        dsi_lane_val |= BIT(5); // DSI_CHANNEL_MODE_SINGLE
    }

    ret = register_write_byte(dev, REG_DSI_LANE, dsi_lane_val);
    if (ret != ESP_OK) return ret;

    // Configure DSI equalization (default values)
    uint8_t dsi_eq_val = (0x2 << 6) | (0x2 << 2); // Default equalization
    ret = register_write_byte(dev, REG_DSI_EQ, dsi_eq_val);
    if (ret != ESP_OK) return ret;

    // Configure DSI clock range (default value)
    ret = register_write_byte(dev, REG_DSI_CLK, 0x1B);
    if (ret != ESP_OK) return ret;

    // Configure LVDS format
    uint8_t lvds_fmt_val = 0;

    if (config.lvds_de_neg_polarity) lvds_fmt_val |= BIT(7);
    if (config.lvds_hs_neg_polarity) lvds_fmt_val |= BIT(6);
    if (config.lvds_vs_neg_polarity) lvds_fmt_val |= BIT(5);

    if (config.lvds_link_config) lvds_fmt_val |= BIT(4); // A-only

    lvds_fmt_val |= (config.lvds_bpp_mode & 0xF);

    ret = register_write_byte(dev, REG_LVDS_FMT, lvds_fmt_val);
    if (ret != ESP_OK) return ret;

    // Configure LVDS VCOM (default values)
    uint8_t lvds_vcom_val = BIT(6) | BIT(4) | (0x2 << 2) | 0x2;
    ret = register_write_byte(dev, REG_LVDS_VCOM, lvds_vcom_val);
    if (ret != ESP_OK) return ret;

    // Configure LVDS lanes (default values)
    uint8_t lvds_lane_val = 0;
    ret = register_write_byte(dev, REG_LVDS_LANE, lvds_lane_val);
    if (ret != ESP_OK) return ret;

    // Configure LVDS common mode (default values)
    uint8_t lvds_cm_val = (0x1 << 4) | 0x1;
    ret = register_write_byte(dev, REG_LVDS_CM, lvds_cm_val);
    if (ret != ESP_OK) return ret;

    return ESP_OK;
}

static esp_err_t configure_video_timing(i2c_master_dev_handle_t dev, sn65dsi83_config_t config)
{
    // Active line length
    esp_err_t ret;
    ret = register_write_byte(dev, REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW, config.active_line_length & 0xFF);
    if (ret != ESP_OK) return ret;

    ret = register_write_byte(dev, REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH, (config.active_line_length >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;

    // Vertical display size
    ret = register_write_byte(dev, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW, config.vertical_display_size & 0xFF);
    if (ret != ESP_OK) return ret;

    ret = register_write_byte(dev, REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH, (config.vertical_display_size >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;

    // Sync delay
    ret = register_write_byte(dev, REG_VID_CHA_SYNC_DELAY_LOW, config.sync_delay & 0xFF);
    if (ret != ESP_OK) return ret;

    ret = register_write_byte(dev, REG_VID_CHA_SYNC_DELAY_HIGH, (config.sync_delay >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;

    // HSYNC pulse width
    ret = register_write_byte(dev, REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW, config.hsync_pulse_width  & 0xFF);
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev, REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH, (config.hsync_pulse_width >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;

    // VSYNC pulse width
    ret = register_write_byte(dev, REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW, config.vsync_pulse_width & 0xFF);
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev, REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH, (config.vsync_pulse_width >> 8) & 0xFF);
    if (ret != ESP_OK) return ret;

    // Back porch
    ret = register_write_byte(dev, REG_VID_CHA_HORIZONTAL_BACK_PORCH, config.h_back_porch & 0xFF);
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev, REG_VID_CHA_HORIZONTAL_BACK_PORCH, config.v_back_porch & 0xFF);
    if (ret != ESP_OK) return ret;

    // Front porch
    ret = register_write_byte(dev, REG_VID_CHA_HORIZONTAL_FRONT_PORCH, config.h_front_porch & 0xFF);
    if (ret != ESP_OK) return ret;
    ret = register_write_byte(dev, REG_VID_CHA_HORIZONTAL_FRONT_PORCH, config.v_front_porch & 0xFF);
    if (ret != ESP_OK) return ret;

    // Disable test pattern
    ret = register_write_byte(dev, REG_VID_CHA_TEST_PATTERN, 0x00);
    if (ret != ESP_OK) return ret;

    return ret;
}

static esp_err_t configure_pll(i2c_master_dev_handle_t dev, sn65dsi83_config_t config)
{
    ESP_LOGI(TAG, "Configuring PLL");

    esp_err_t ret;
    uint8_t lvds_pll_val = (0x3 << 1) | BIT(0);
    ret = register_write_byte(dev, REG_RC_LVDS_PLL, lvds_pll_val);
    if (ret != ESP_OK) return ret;

    uint8_t dsi_clk_val = ((config.dsi_clk_divider & 0x1F) << 3) |
                         (config.dsi_refclk_multiplier & 0x3);
    ret = register_write_byte(dev, REG_RC_DSI_CLK, dsi_clk_val);
    if (ret != ESP_OK) return ret;

    ret = register_write_byte(dev, REG_RC_PLL_EN, BIT(0));
    if (ret != ESP_OK) return ret;

    bool pll_locked = false;
    for (int i = 0; i < 10; i++) {
        uint8_t pll_status;
        if (register_read(dev, REG_RC_LVDS_PLL, &pll_status, 1)) {
            if (pll_status & BIT(7)) {
                pll_locked = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (!pll_locked) {
        ESP_LOGI(TAG, "PLL failed to lock");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PLL locked successfully");
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

static void sn65dsi83_state_machine(i2c_master_dev_handle_t dev, sn65dsi83_config_t config)
{
    static uint8_t val;

    switch (sn65_ctx.state) {
    case INIT_POWER_ON:
        ESP_LOGI(TAG, "Power on sequence started");
        sn65_ctx.state = INIT_DSI_LANE_STATE;
        break;

    case INIT_DSI_LANE_STATE:
        ESP_LOGI(TAG, "DSI lanes: CLK=HS, DATA=LP11 (should be set by DSI host)");
        sn65_ctx.state = INIT_EN_LOW;
        break;

    case INIT_EN_LOW:
        ESP_LOGI(TAG, "Setting EN pin LOW");
        en_assert(SN65_EN_GPB3, 0);
        sn65_ctx.state = INIT_EN_HIGH;
        break;

    case INIT_EN_HIGH:
        ESP_LOGI(TAG, "Setting EN pin HIGH");
        en_assert(SN65_EN_GPB3, 1);
        sn65_ctx.state = INIT_WRITE_REGS;
        break;

    case INIT_WRITE_REGS:
        ESP_LOGI(TAG, "Writing configuration registers...");
        sn65dsi83_configure(dev, config);
        configure_video_timing(dev, config);
        sn65_ctx.state = INIT_PLL_ENABLE;
        break;

    case INIT_PLL_ENABLE:
        ESP_LOGI(TAG, "Enable PLL");
        //sn65_write_reg(0x0D, 0x01); // PLL_EN bit
        configure_pll(dev, config);
        vTaskDelay(10);
        sn65_ctx.state = INIT_SOFT_RESET;
        break;

    case INIT_SOFT_RESET:
        ESP_LOGI(TAG, "Perform soft reset");
        //sn65_write_reg(0x09, 0x01); // SOFT_RESET
        esp_err_t ret = register_write_byte(dev, REG_RC_RESET, BIT(0));
        if (ret != ESP_OK) ESP_LOGE(TAG, "Soft Reset error %s", ret);
        vTaskDelay(10);
        sn65_ctx.state = INIT_START_STREAM;
        break;

    case INIT_START_STREAM:
        ESP_LOGI(TAG, "Switch DSI lanes to HS and start video stream");
        // Here you would trigger DSI host transmission
        vTaskDelay(5);
        sn65_ctx.state = INIT_VERIFY_REGS;
        break;

    case INIT_VERIFY_REGS:
        ESP_LOGI(TAG, "Verifying register values...");
        ESP_LOGI(TAG, "PLL_EN reg=0x%02X", val);
        sn65_ctx.state = INIT_CLEAR_ERRORS;
        break;

    case INIT_CLEAR_ERRORS:
        ESP_LOGI(TAG, "Clearing error registers...");
        //sn65_write_reg(0xE5, 0xFF);
        vTaskDelay(1);
        sn65_ctx.state = INIT_VERIFY_ERRORS;
        break;

    case INIT_VERIFY_ERRORS:
        //sn65_read_reg(0xE5, &val);
        if (val != 0x00) {
            ESP_LOGW(TAG, "Error detected (0xE5=0x%02X). Restarting init.", val);
            sn65_ctx.state = INIT_DSI_LANE_STATE;
        } else {
            sn65_ctx.state = INIT_DONE;
        }
        break;

    case INIT_DONE:
        ESP_LOGI(TAG, "SN65DSI83 initialization complete!");
        sn65_ctx.initialized = true;
        break;

    case INIT_ERROR:
    default:
        ESP_LOGE(TAG, "Initialization failed!");
        break;
    }
}

void run_i2c()
{
    // Init Expander channel
    mcp23017_init();
    
    //en_assert(11, 0);
    //bool out_level;
    //mcp23017_read_pin(11, &out_level);

    //Set brightness high
    en_assert(7, 1);
    //Set SN enable to work
    en_assert(11, 1);
    //LCD standby control (turn 3.3v ON with HIGH)
    en_assert(10, 1); 
    //LCD Reset
    en_assert(9, 1); 

    for(int i = 0; i < 5; i++) {

        en_assert(5, 1);
        vTaskDelay(pdMS_TO_TICKS(3));
        en_assert(5, 0);
        vTaskDelay(pdMS_TO_TICKS(4));
    }
    
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    write_all_regs_zero(dev_handle, 0x00, 0xFF);
    // Dump all posibly register 0x09-0xE5
    dump_registers(dev_handle, 0x00, 0xFF);

    esp_err_t ret;

    ret = sn65dsi83_enable_test_pattern(dev_handle);
    ret = register_write_byte(dev_handle, 0x18, 0x10);// Enable channel A
    ret = register_write_byte(dev_handle, 0x19, 0x00);// LVDS Voltage level 
    //vTaskDelay(pdMS_TO_TICKS(5000));
    //ret = register_write_byte(dev_handle, 0x3C, 0x00);
    //ret = register_write_byte(dev_handle, 0x3C, 0x10);
    if (ret != ESP_OK) {
        printf("I2C write failed, err=0x%x\n", ret);
    }
    ret = register_write_byte(dev_handle, REG_RC_RESET, BIT(0));
    if (ret != ESP_OK) ESP_LOGE(TAG, "Soft Reset error %s", ret);
    vTaskDelay(10);


    ////while(1) {
    ////    sn65dsi83_state_machine(dev_handle, default_config);
    ////    vTaskDelay(pdMS_TO_TICKS(100));
    ////}
    //Again dump after TEST PATTERN
    while(1) {
            ret = register_write_byte(dev_handle, 0xE5, 0x01);//Clear error
        dump_registers(dev_handle, 0x00, 0xFF);
        vTaskDelay(pdMS_TO_TICKS(2000));


       // static int innit = 0;
       // if (innit == 1) {
       //     ret = register_write_byte(dev_handle, 0xE5, 0x01);//Clear error
       //     innit = 2;
       // }
       // if (0 == innit) {
       //     uint8_t dd=0;
       //     register_read(dev_handle, 0x0A, &dd, 1);
       //     if(0x80 & dd)
       //         innit = 1;
       //  }
    }
    /* Read the SN65DSI83's PLL register, it should have the value 0x83(locked) */
    //ESP_ERROR_CHECK(register_read(dev_handle, 0x0a, data, 1));
    //ESP_LOGI(TAG, "PLL is locked = %X", data[0]); 


}
