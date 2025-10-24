#pragma once
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_lcd_mipi_dsi.h"

/* Register definitions */

#define REG_ID(n)                       (0x00 + (n))
#define REG_RC_RESET                    0x09
#define REG_RC_LVDS_PLL                 0x0a
#define REG_RC_DSI_CLK                  0x0b
#define REG_RC_PLL_EN                   0x0d
#define REG_DSI_LANE                    0x10
#define REG_DSI_EQ                      0x11
#define REG_DSI_CLK                     0x12
#define REG_LVDS_FMT                    0x18
#define REG_LVDS_VCOM                   0x19
#define REG_LVDS_LANE                   0x1a
#define REG_LVDS_CM                     0x1b
#define REG_VID_CHA_ACTIVE_LINE_LENGTH_LOW 0x20
#define REG_VID_CHA_ACTIVE_LINE_LENGTH_HIGH 0x21
#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_LOW 0x24
#define REG_VID_CHA_VERTICAL_DISPLAY_SIZE_HIGH 0x25
#define REG_VID_CHA_SYNC_DELAY_LOW      0x28
#define REG_VID_CHA_SYNC_DELAY_HIGH     0x29
#define REG_VID_CHA_HSYNC_PULSE_WIDTH_LOW 0x2c
#define REG_VID_CHA_HSYNC_PULSE_WIDTH_HIGH 0x2d
#define REG_VID_CHA_VSYNC_PULSE_WIDTH_LOW 0x30
#define REG_VID_CHA_VSYNC_PULSE_WIDTH_HIGH 0x31
#define REG_VID_CHA_HORIZONTAL_BACK_PORCH 0x34
#define REG_VID_CHA_VERTICAL_BACK_PORCH 0x36
#define REG_VID_CHA_HORIZONTAL_FRONT_PORCH 0x38
#define REG_VID_CHA_VERTICAL_FRONT_PORCH 0x3a
#define REG_VID_CHA_TEST_PATTERN        0x3c
#define REG_IRQ_GLOBAL                  0xe0
#define REG_IRQ_EN                      0xe1
#define REG_IRQ_STAT                    0xe5

/* Configuration structure */
typedef struct sn65dsi83_config {
    /* DSI configuration */
    uint8_t dsi_lanes;
    uint8_t dsi_channel_mode;
    uint8_t dsi_clk_divider;
    uint8_t dsi_refclk_multiplier;

    /* LVDS configuration */
    uint8_t lvds_link_config;
    uint8_t lvds_bpp_mode;
    bool lvds_de_neg_polarity;
    bool lvds_hs_neg_polarity;
    bool lvds_vs_neg_polarity;

    /* Video timing */
    uint16_t active_line_length;
    uint16_t vertical_display_size;
    uint16_t sync_delay;
    uint16_t hsync_pulse_width;
    uint16_t vsync_pulse_width;
    uint8_t h_back_porch;
    uint8_t v_back_porch;
    uint8_t h_front_porch;
    uint8_t v_front_porch;
} sn65dsi83_config_t;

/*
    // Example initialization
//sn65dsi83_panel_config_t;
sn65dsi83_panel_config_t sn65dsi83_800_1280_60hz_config = {
    .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
    .dpi_clock_freq_mhz = 80,
    .virtual_channel = 0,
    .pixel_format = PIXEL_FORMAT_RGB888, // replace with your enum or define
    .num_fbs = 1,
    .video_timing = {
        .h_size = 1280,
        .v_size = 800,
        .hsync_back_porch = 88,
        .hsync_pulse_width = 20,
        .hsync_front_porch = 72,
        .vsync_back_porch = 23,
        .vsync_pulse_width = 10,
        .vsync_front_porch = 15,
    },
    .flags = {
        .use_dma2d = true,
    }
};
*/
typedef struct {
    uint8_t dpi_clk_src;        // Source for DPI clock, e.g., default MIPI DSI source
    uint32_t dpi_clock_freq_mhz;// DPI clock frequency in MHz
    uint8_t virtual_channel;    // DSI virtual channel number
    uint32_t pixel_format;      // Pixel format, e.g., RGB888
    uint8_t num_fbs;            // Number of framebuffers
    struct {
        uint16_t h_size;            // Horizontal resolution (pixels)
        uint16_t v_size;            // Vertical resolution (pixels)
        uint16_t hsync_back_porch;  // Horizontal back porch (pixels)
        uint16_t hsync_pulse_width; // HSYNC pulse width (pixels)
        uint16_t hsync_front_porch; // Horizontal front porch (pixels)
        uint16_t vsync_back_porch;  // Vertical back porch (lines)
        uint16_t vsync_pulse_width; // VSYNC pulse width (lines)
        uint16_t vsync_front_porch; // Vertical front porch (lines)
    } video_timing;
    struct {
        bool use_dma2d;          // Flag to indicate using DMA2D for rendering
    } flags;
} sn65dsi83_panel_config_t;

// All initialization states
typedef enum {
    INIT_POWER_ON = 0,
    INIT_DSI_LANE_STATE,
    INIT_EN_LOW,
    INIT_EN_HIGH,
    INIT_WRITE_REGS,
    INIT_PLL_ENABLE,
    INIT_SOFT_RESET,
    INIT_START_STREAM,
    INIT_VERIFY_REGS,
    INIT_CLEAR_ERRORS,
    INIT_VERIFY_ERRORS,
    INIT_DONE,
    INIT_ERROR
} sn65dsi83_state_t;


typedef struct {
    sn65dsi83_state_t state;
    bool initialized;
} sn65dsi83_ctx_t; // ALL states and init or no.

typedef struct {
    uint8_t bus_id;             // DSI bus ID
    uint8_t num_data_lanes;     // Number of active DSI data lanes
    uint8_t phy_clk_src;        // PHY clock source, e.g., default MIPI DSI source
    uint32_t lane_bit_rate_mbps;// Bit rate per lane in Mbps
} sn65dsi83_dsi_bus_config_t;

//void sn65dsi83_state_machine(i2c_master_dev_handle_t *dev, sn65dsi83_config config);

// Initialize I2C bus and run to dump registers
void run_i2c();
