// SPDX-License-Identifier: Apache-2.0
#include "rb_board.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Board wiring only; this component has no dependency on another application. */
enum {
    SDA = 23, SCL = 24, LCD_CLK = 8, LCD_MOSI = 10, LCD_CS = 7, LCD_DC = 25,
    TOUCH_INT = 0, PCA_ADDRESS = 0x21,
    LCD_RESET = 1u << 0, TOUCH_RESET = 1u << 1, BACKLIGHT = 1u << 5,
    CAN_STANDBY = (1u << 8) | (1u << 9), LEDS_OFF = 7u << 10,
    OUTPUTS = LCD_RESET | TOUCH_RESET | BACKLIGHT | CAN_STANDBY | LEDS_OFF
};
static const char *TAG = "rb_board";
static i2c_master_dev_handle_t pca;
static esp_lcd_touch_handle_t touch;
static uint16_t output_levels;
static bool started;
static bool ready;

#if CONFIG_REBUDDY_DISPLAY_STATS
/* These callbacks and the log timer all run in the LVGL task. No ISR logging. */
static struct {
    int64_t since_us, render_start_us;
    uint64_t pixels, render_us;
    uint32_t frames;
} display_stats;
static void display_event(lv_event_t *event)
{
    switch (lv_event_get_code(event)) {
    case LV_EVENT_RENDER_START:
        display_stats.render_start_us=esp_timer_get_time();
        break;
    case LV_EVENT_RENDER_READY:
        display_stats.render_us+=(uint64_t)(esp_timer_get_time()-display_stats.render_start_us);
        ++display_stats.frames;
        break;
    case LV_EVENT_FLUSH_START: {
        const lv_area_t *area=lv_event_get_param(event);
        display_stats.pixels+=(uint64_t)lv_area_get_width(area)*lv_area_get_height(area);
        break;
    }
    default: break;
    }
}
static void display_log(lv_timer_t *timer)
{
    (void)timer;
    int64_t now=esp_timer_get_time();
    double seconds=(now-display_stats.since_us)*.000001;
    if (seconds<=0) return;
    ESP_LOGI("rb_display", "redraw=%.1f/s render_submit=%.2fms pixels/redraw=%lu queued=%.1fKiB/s",
             display_stats.frames/seconds,
             display_stats.frames?display_stats.render_us*.001/display_stats.frames:0,
             (unsigned long)(display_stats.frames?display_stats.pixels/display_stats.frames:0),
             display_stats.pixels*2.0/1024/seconds);
    display_stats.frames=0; display_stats.pixels=0; display_stats.render_us=0;
    display_stats.since_us=now;
}
#endif

/* PCA9535 paired registers: inputs 0, outputs 2, polarity 4, direction 6. */
static esp_err_t pca_write(uint8_t reg, uint16_t value)
{
    uint8_t packet[] = {reg, (uint8_t)value, (uint8_t)(value >> 8)};
    return i2c_master_transmit(pca, packet, sizeof(packet), 20);
}
static esp_err_t set_output(uint16_t mask, bool high)
{
    uint16_t next = high ? output_levels | mask : output_levels & (uint16_t)~mask;
    esp_err_t err = pca_write(2, next);
    if (err == ESP_OK) output_levels = next;
    return err;
}
static void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

esp_err_t rb_board_can1_standby(bool standby)
{
    if (!ready) return ESP_ERR_INVALID_STATE;
    return set_output(1u<<8,standby);
}

bool rb_board_display_lock(uint32_t timeout_ms) { return lvgl_port_lock(timeout_ms); }
void rb_board_display_unlock(void) { lvgl_port_unlock(); }

static void read_touch(lv_indev_t *indev, lv_indev_data_t *data)
{
    static lv_point_t last;
    static rb_primary_touch_t primary;
    esp_lcd_touch_point_data_t points[CONFIG_ESP_LCD_TOUCH_MAX_POINTS] = {0};
    uint8_t ids[CONFIG_ESP_LCD_TOUCH_MAX_POINTS];
    uint8_t count = 0;
    data->point = last;
    data->state = LV_INDEV_STATE_RELEASED;
    if (esp_lcd_touch_read_data(touch) != ESP_OK) {
        rb_primary_touch_select(&primary, NULL, 0, false);
        /* INDEV_RESET cancels the gesture instead of synthesizing a tap. */
        lv_indev_reset(indev, NULL);
        return;
    }
    if (esp_lcd_touch_get_data(touch, points, &count, CONFIG_ESP_LCD_TOUCH_MAX_POINTS) != ESP_OK) {
        rb_primary_touch_select(&primary, NULL, 0, false);
        lv_indev_reset(indev, NULL);
        return;
    }
    for (unsigned i=0;i<count;++i) ids[i]=points[i].track_id;
    int selected=rb_primary_touch_select(&primary, ids, count, true);
    if (selected==-2) { lv_indev_reset(indev, NULL); return; }
    if (selected>=0) {
        esp_lcd_touch_point_data_t point=points[selected];
        int16_t mapped_x, mapped_y;
        if (!rb_touch_map(point.x, point.y, &mapped_x, &mapped_y)) {
            rb_primary_touch_select(&primary, NULL, 0, false);
            lv_indev_reset(indev, NULL);
            return;
        }
        last.x = mapped_x;
        last.y = mapped_y;
        data->point = last;
        data->state = LV_INDEV_STATE_PRESSED;
    }
}

esp_err_t rb_board_init(void)
{
    if (started) return ESP_ERR_INVALID_STATE;
    started = true;
    i2c_master_bus_handle_t bus;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0, .sda_io_num = SDA, .scl_io_num = SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &bus), TAG, "I2C bus");
    const i2c_device_config_t pca_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = PCA_ADDRESS,
        .scl_speed_hz = 100000
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &pca_config, &pca), TAG, "PCA9535");
    /* Program inactive outputs before enabling drivers: CAN stays in standby. */
    output_levels = LCD_RESET | TOUCH_RESET | CAN_STANDBY | LEDS_OFF;
    ESP_RETURN_ON_ERROR(pca_write(2, output_levels), TAG, "Output levels");
    ESP_RETURN_ON_ERROR(pca_write(4, 0), TAG, "Input polarity");
    ESP_RETURN_ON_ERROR(pca_write(6, (uint16_t)~OUTPUTS), TAG, "I/O directions");

    ESP_RETURN_ON_ERROR(set_output(LCD_RESET, false), TAG, "LCD reset");
    delay_ms(20);
    ESP_RETURN_ON_ERROR(set_output(LCD_RESET, true), TAG, "LCD release");
    delay_ms(120);
    const spi_bus_config_t spi = {
        .sclk_io_num = LCD_CLK, .mosi_io_num = LCD_MOSI, .miso_io_num = -1,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 320 * 40 * 2
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &spi, SPI_DMA_CH_AUTO), TAG, "LCD SPI");
    esp_lcd_panel_io_handle_t lcd_io;
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = LCD_CS, .dc_gpio_num = LCD_DC, .spi_mode = 0,
        .pclk_hz = CONFIG_REBUDDY_LCD_SPI_MHZ * 1000000,
        .trans_queue_depth = 10, .lcd_cmd_bits = 8, .lcd_param_bits = 8
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                               &io_config, &lcd_io), TAG, "LCD IO");
    esp_lcd_panel_handle_t panel;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = -1, .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE, .bits_per_pixel = 16
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &panel), TAG, "ST7789");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), TAG, "Panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), TAG, "Panel init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel, true), TAG, "Panel inversion");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), TAG, "Panel on");

    /* GT911 0x5D selection: hold INT low across reset, then return it to input.
     * Reset is on the expander, so the public touch driver must skip GPIO reset. */
    const gpio_config_t int_output = {
        .pin_bit_mask = 1ULL << TOUCH_INT, .mode = GPIO_MODE_OUTPUT
    };
    ESP_RETURN_ON_ERROR(set_output(TOUCH_RESET, false), TAG, "Touch reset");
    ESP_RETURN_ON_ERROR(gpio_config(&int_output), TAG, "Touch INT output");
    ESP_RETURN_ON_ERROR(gpio_set_level(TOUCH_INT, 0), TAG, "Touch address");
    delay_ms(20);
    ESP_RETURN_ON_ERROR(set_output(TOUCH_RESET, true), TAG, "Touch release");
    delay_ms(60);
    const gpio_config_t int_input = {
        .pin_bit_mask = 1ULL << TOUCH_INT, .mode = GPIO_MODE_INPUT
    };
    ESP_RETURN_ON_ERROR(gpio_config(&int_input), TAG, "Touch INT input");
    esp_lcd_panel_io_handle_t touch_io;
    esp_lcd_panel_io_i2c_config_t touch_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    touch_io_config.transaction_timeout_ms = 20;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bus, &touch_io_config, &touch_io), TAG, "Touch IO");
    const esp_lcd_touch_config_t touch_config = {
        .x_max = 240, .y_max = 320, .rst_gpio_num = -1, .int_gpio_num = -1
    };
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_gt911(touch_io, &touch_config, &touch), TAG, "GT911");

    lvgl_port_cfg_t lvgl_config = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_config.task_max_sleep_ms = 10;
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_config), TAG, "LVGL");
    const lvgl_port_display_cfg_t display_config = {
        .io_handle = lcd_io, .panel_handle = panel, .buffer_size = 320 * 40,
        .double_buffer = true,
        .hres = 320, .vres = 240, .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {.swap_xy = true, .mirror_x = true},
        .flags.buff_dma = true
    };
    lv_display_t *display = lvgl_port_add_disp(&display_config);
    ESP_RETURN_ON_FALSE(display, ESP_ERR_NO_MEM, TAG, "LVGL display");
    ESP_RETURN_ON_FALSE(lvgl_port_lock(0), ESP_ERR_TIMEOUT, TAG, "LVGL lock");
    /* 50ms dance frames align with this 10ms display check. Input stays at 20ms. */
    lv_timer_set_period(lv_display_get_refr_timer(display), 10);
#if CONFIG_REBUDDY_DISPLAY_STATS
    display_stats.since_us=esp_timer_get_time();
    lv_display_add_event_cb(display,display_event,LV_EVENT_RENDER_START,NULL);
    lv_display_add_event_cb(display,display_event,LV_EVENT_RENDER_READY,NULL);
    lv_display_add_event_cb(display,display_event,LV_EVENT_FLUSH_START,NULL);
    lv_timer_create(display_log,5000,NULL);
#endif
    lv_indev_t *pointer = lv_indev_create();
    if (pointer) {
        lv_indev_set_type(pointer, LV_INDEV_TYPE_POINTER);
        lv_indev_set_disp(pointer, display);
        lv_indev_set_read_cb(pointer, read_touch);
        lv_timer_set_period(lv_indev_get_read_timer(pointer), 20);
    }
    lvgl_port_unlock();
    ESP_RETURN_ON_FALSE(pointer, ESP_ERR_NO_MEM, TAG, "Touch pointer");
    ESP_RETURN_ON_ERROR(set_output(BACKLIGHT, true), TAG, "Backlight");
    ESP_LOGI(TAG, "Touch-only board ready: 320x240, GT911, PCA9535 outputs only; no key/GPIO-stop input; CAN in standby");
    ESP_LOGI(TAG, "Display: %dMHz SPI, 2x320x40 RGB565 DMA buffers, 10ms refresh polling / 20ms input",
             CONFIG_REBUDDY_LCD_SPI_MHZ);
    ready=true;
    return ESP_OK;
}
