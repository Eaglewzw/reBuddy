/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <stdint.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t tx_gpio;
    gpio_num_t rx_gpio;
} can_console_bsp_can_pins_t;

typedef struct {
    gpio_num_t sda_gpio;
    gpio_num_t scl_gpio;
} can_console_bsp_i2c_pins_t;

typedef struct {
    gpio_num_t sclk_gpio;
    gpio_num_t mosi_gpio;
    gpio_num_t cs_gpio;
    gpio_num_t dc_gpio;
    gpio_num_t reset_gpio;
} can_console_bsp_lcd_pins_t;

typedef struct {
    gpio_num_t output_gpio;
} can_console_bsp_pwm_pins_t;

typedef struct {
    gpio_num_t data_gpio;
} can_console_bsp_led_strip_pins_t;

typedef struct {
    gpio_num_t touch_int_gpio;
    gpio_num_t touch_reset_gpio;
    gpio_num_t estop_gpio;
    gpio_num_t ioexp_int_gpio;
} can_console_bsp_gpio_pins_t;

typedef struct {
    uint16_t encoder_a_pin;
    uint16_t encoder_b_pin;
    uint16_t encoder_push_pin;
    uint16_t enter_pin;
    uint16_t back_pin;

    uint16_t can1_stb_pin;
    uint16_t can2_stb_pin;

    uint16_t can1_led_pin;
    uint16_t can2_led_pin;
    
    uint16_t error_led_pin;

    uint16_t lcd_backlight_pin;
    uint16_t lcd_reset_pin;

    uint16_t touch_reset_pin;

    uint16_t output_mask;
} can_console_bsp_expander_pins_t;

const can_console_bsp_can_pins_t *can_console_bsp_get_can_pins(uint8_t channel);
const can_console_bsp_i2c_pins_t *can_console_bsp_get_i2c_pins(void);
const can_console_bsp_lcd_pins_t *can_console_bsp_get_lcd_pins(void);
const can_console_bsp_pwm_pins_t *can_console_bsp_get_pwm_pins(void);
const can_console_bsp_led_strip_pins_t *can_console_bsp_get_led_strip_pins(void);
const can_console_bsp_gpio_pins_t *can_console_bsp_get_gpio_pins(void);
const can_console_bsp_expander_pins_t *can_console_bsp_get_expander_pins(void);

#ifdef __cplusplus
}
#endif
