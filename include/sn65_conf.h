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

void show_regs();
void sn_chip_en(bool en);
void sn_pll_en(bool en);
void sn_soft_reset(bool set);
void sn_soft_reset(bool set);
bool is_sn_pll_locked();
void pll_checker_task(void *arg);

// Initialize I2C bus and run to dump registers

esp_err_t clear_error_reg();
void sn65dsi_fill_regs();
void sn65dsi_deinit();
void sn65dsi_init_and_enable();
