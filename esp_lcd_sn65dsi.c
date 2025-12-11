/*
 *  Riverdi Sp. z o.o 
 *  DATE: December 2025
 */

#include "soc/soc_caps.h"

#if SOC_MIPI_DSI_SUPPORTED
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "mipi_dsi_priv.h"
#include "esp_lcd_sn65dsi.h"
#include "sn65_conf.h"

#define ESP_LCD_SN65DSI_VER_MAJOR 0
#define ESP_LCD_SN65DSI_VER_MINOR 0
#define ESP_LCD_SN65DSI_VER_PATCH 1

typedef struct {
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    const sn65dsi_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
    uint8_t lane_num;
    struct {
        unsigned int reset_level: 1;
    } flags;
    // To save the original functions of MIPI DPI panel
    esp_err_t (*del)(esp_lcd_panel_t *panel);
    esp_err_t (*init)(esp_lcd_panel_t *panel);
} sn65dsi_panel_t;

static const char *TAG = "sn65dsi riv10inch";
static int init_t = 0;

static mipi_dsi_hal_context_t *s_hal = NULL;

static esp_err_t panel_sn65dsi_send_init_cmds(sn65dsi_panel_t *sn65dsi_panel);

static esp_err_t panel_sn65dsi_del(esp_lcd_panel_t *panel);
static esp_err_t panel_sn65dsi_init(esp_lcd_panel_t *panel);
static esp_err_t panel_sn65dsi_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_sn65dsi_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_sn65dsi_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);

static esp_err_t configure_mode(bool en)
{
    mipi_dsi_brg_ll_enable_dpi_output(s_hal->bridge, false);
    mipi_dsi_brg_ll_update_dpi_config(s_hal->bridge);

    if(en) {
        mipi_dsi_host_ll_set_clock_lane_state(
            s_hal->host,
            MIPI_DSI_LL_CLOCK_LANE_STATE_HS
        );
        mipi_dsi_host_ll_enable_video_mode(s_hal->host, false);
        mipi_dsi_host_ll_enable_cmd_ack(s_hal->host, false);

        mipi_dsi_brg_ll_enable_dpi_output(s_hal->bridge, false);

    } else { // Prepare for data stream

        mipi_dsi_host_ll_set_clock_lane_state(
            s_hal->host,
            MIPI_DSI_LL_CLOCK_LANE_STATE_HS
        );
        mipi_dsi_host_ll_enable_video_mode(s_hal->host, true);

        mipi_dsi_host_ll_enable_cmd_ack(s_hal->host, false);
        mipi_dsi_brg_ll_enable_dpi_output(s_hal->bridge, true);
    }

    mipi_dsi_brg_ll_update_dpi_config(s_hal->bridge);

    return ESP_OK;
}

esp_err_t esp_lcd_new_panel_sn65dsi(const esp_lcd_panel_io_handle_t io,
    const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    ESP_LOGI(TAG, "version: %d.%d.%d", ESP_LCD_SN65DSI_VER_MAJOR,
        ESP_LCD_SN65DSI_VER_MINOR, ESP_LCD_SN65DSI_VER_PATCH);

    ESP_RETURN_ON_FALSE(io && panel_dev_config &&
        ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid arguments");

    sn65dsi_vendor_config_t *vendor_config =
        (sn65dsi_vendor_config_t *)panel_dev_config->vendor_config;

    ESP_RETURN_ON_FALSE(vendor_config && vendor_config->mipi_config.dpi_config &&
        vendor_config->mipi_config.dsi_bus, ESP_ERR_INVALID_ARG, TAG, "invalid vendor config");

    // Assigne to global pointer to controll bus
    s_hal = &vendor_config->mipi_config.dsi_bus->hal;

    esp_err_t ret = ESP_OK;
    sn65dsi_panel_t *sn65dsi = (sn65dsi_panel_t *)calloc(1, sizeof(sn65dsi_panel_t));
    ESP_RETURN_ON_FALSE(sn65dsi, ESP_ERR_NO_MEM, TAG, "no mem for sn65dsi bridge panel");

    sn65dsi->io = io;
    sn65dsi->init_cmds = vendor_config->init_cmds;
    sn65dsi->init_cmds_size = vendor_config->init_cmds_size;
    sn65dsi->lane_num = vendor_config->mipi_config.lane_num;
    //sn65dsi->reset_gpio_num = panel_dev_config->reset_gpio_num;
    sn65dsi->flags.reset_level = panel_dev_config->flags.reset_active_high;
    sn65dsi->madctl_val = 0x01;

    // Create MIPI DPI panel
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(vendor_config->mipi_config.dsi_bus,
        vendor_config->mipi_config.dpi_config, ret_panel),
        err, TAG, "create MIPI DPI panel failed");
    ESP_LOGD(TAG, "new MIPI DPI panel @%p", *ret_panel);

    // Save the original functions of MIPI DPI panel
    sn65dsi->del = (*ret_panel)->del;
    sn65dsi->init = (*ret_panel)->init;
    // Overwrite the functions of MIPI DPI panel
    (*ret_panel)->del = panel_sn65dsi_del;
    (*ret_panel)->init = panel_sn65dsi_init;
    (*ret_panel)->reset = panel_sn65dsi_reset;
    (*ret_panel)->mirror = panel_sn65dsi_mirror;
    (*ret_panel)->invert_color = panel_sn65dsi_invert_color;
    (*ret_panel)->user_data = sn65dsi;
    ESP_LOGD(TAG, "new sn65dsi panel @%p", sn65dsi);

    return ESP_OK;

err:
    if (sn65dsi) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(sn65dsi);
    }
    return ret;
}

static esp_err_t panel_sn65dsi_send_init_cmds(sn65dsi_panel_t *sn65dsi)
{
    configure_mode(true);

    sn65dsi_init_and_enable();

    sn_chip_en(false);
    vTaskDelay(pdMS_TO_TICKS(10));

    sn_chip_en(true);
    vTaskDelay(pdMS_TO_TICKS(20));
    // configure sn65 regs
    sn65dsi_fill_regs();

    return ESP_OK;
}

static esp_err_t panel_sn65dsi_del(esp_lcd_panel_t *panel)
{
    sn65dsi_panel_t *sn65dsi = (sn65dsi_panel_t *)panel->user_data;

    // Delete MIPI DPI panel
    ESP_RETURN_ON_ERROR(sn65dsi->del(panel), TAG, "del sn65dsi panel failed");
    if (sn65dsi->reset_gpio_num >= 0) {
        gpio_reset_pin(sn65dsi->reset_gpio_num);
    }
    // REMOVE sn65, mcp23017, Brightness and disp reset to low,
    sn65dsi_deinit();
    ESP_LOGI(TAG, "DELETE SN65,MCP, DIPS BRIGHTNESS AND RESET TO LOW");
    
    ESP_LOGD(TAG, "del sn65dsi panel @%p", sn65dsi);
    free(sn65dsi);

    return ESP_OK;
}

static esp_err_t panel_sn65dsi_init(esp_lcd_panel_t *panel)
{
    ESP_LOGI(TAG, "PANEL INIT %d time", init_t);

    sn65dsi_panel_t *sn65dsi = (sn65dsi_panel_t *)panel->user_data;

    ESP_RETURN_ON_ERROR(panel_sn65dsi_send_init_cmds(sn65dsi), TAG, "send init commands failed");

    ESP_RETURN_ON_ERROR(sn65dsi->init(panel), TAG, "init MIPI DPI panel failed");

    configure_mode(true);
    vTaskDelay(pdMS_TO_TICKS(100));

    // SN PLL ENABLE
    sn_pll_en(true);
    vTaskDelay(pdMS_TO_TICKS(12));

    // Perform soft reset 
    sn_soft_reset(true);
    vTaskDelay(pdMS_TO_TICKS(12));

    bool k = is_sn_pll_locked();
    ESP_LOGI(TAG, "PLL LOCK %d", k); 

    configure_mode(false);

    clear_error_reg();
    //show_regs();

    k = is_sn_pll_locked();
    ESP_LOGI(TAG, "PLL LOCK %d", k); 

    return ESP_OK;
}

static esp_err_t panel_sn65dsi_reset(esp_lcd_panel_t *panel)
{
    return ESP_OK;
}

static esp_err_t panel_sn65dsi_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    return ESP_OK;
}

static esp_err_t panel_sn65dsi_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    return ESP_OK;
}
#endif
