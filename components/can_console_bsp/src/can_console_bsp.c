/* SPDX-License-Identifier: Apache-2.0 */
#include <stddef.h>

#include "can_console_bsp.h"

#define PIN(n) ((uint16_t)(1U << (n)))

/* Board channel order: 0=CAN1, 1=CAN2. TX/RX follow the transceiver
 * TXD/RXD nets: MTDO/MTDI=GPIO5/3, MTMS/MTCK=GPIO2/4. */
static const can_console_bsp_can_pins_t s_can_pins[2] = {
    { GPIO_NUM_5, GPIO_NUM_3 }, 
    { GPIO_NUM_2, GPIO_NUM_4 },
};

static const can_console_bsp_i2c_pins_t s_i2c_pins = {
    .sda_gpio=GPIO_NUM_23, 
    .scl_gpio=GPIO_NUM_24
};

static const can_console_bsp_lcd_pins_t s_lcd_pins = {
    GPIO_NUM_8, GPIO_NUM_10, GPIO_NUM_7, GPIO_NUM_25, GPIO_NUM_NC
};

static const can_console_bsp_pwm_pins_t s_pwm_pins = {    .output_gpio = GPIO_NUM_1,
};

static const can_console_bsp_gpio_pins_t s_gpio_pins = {
    .touch_int_gpio = GPIO_NUM_0,
    .touch_reset_gpio = GPIO_NUM_NC,
    .estop_gpio = GPIO_NUM_28,
    .ioexp_int_gpio = GPIO_NUM_9,
};

static const can_console_bsp_led_strip_pins_t s_led_strip_pins = {
    .data_gpio = GPIO_NUM_1,
};

static const can_console_bsp_expander_pins_t s_expander_pins = {
    .encoder_a_pin      = PIN(2),
    .encoder_b_pin      = PIN(3),
    .encoder_push_pin   = PIN(4),
    .enter_pin          = PIN(6),
    .back_pin            = PIN(7),

    .can1_stb_pin       = PIN(8),
    .can2_stb_pin       = PIN(9),
    .can1_led_pin       = PIN(10),
    .can2_led_pin       = PIN(11),
    .error_led_pin      = PIN(12),

    .lcd_backlight_pin  = PIN(5),
    .lcd_reset_pin      = PIN(0),

    .touch_reset_pin  = PIN(1),

    /*io mode*/
    .output_mask =  PIN(0) | PIN(1) | PIN(5) |
                    PIN(8) | PIN(9) |
                    PIN(10) | PIN(11) | PIN(12),
};

const can_console_bsp_can_pins_t *can_console_bsp_get_can_pins(uint8_t channel)
{
    return channel < 2U ? &s_can_pins[channel] : NULL;
}

const can_console_bsp_i2c_pins_t *can_console_bsp_get_i2c_pins(void)
{
    return &s_i2c_pins;
}

const can_console_bsp_lcd_pins_t *can_console_bsp_get_lcd_pins(void)
{
    return &s_lcd_pins;
}

const can_console_bsp_pwm_pins_t *can_console_bsp_get_pwm_pins(void)
{
    return &s_pwm_pins;
}

const can_console_bsp_led_strip_pins_t *can_console_bsp_get_led_strip_pins(void)
{
    return &s_led_strip_pins;
}

const can_console_bsp_gpio_pins_t *can_console_bsp_get_gpio_pins(void)
{
    return &s_gpio_pins;
}

const can_console_bsp_expander_pins_t *can_console_bsp_get_expander_pins(void)
{
    return &s_expander_pins;
}
