/**
 * BearPi-Pico H3863 on-board LED adapter for the reliable-link sample.
 */
#include "sle_rl_board_led.h"

#include "gpio.h"
#include "pinctrl.h"

static gpio_level_t sle_rl_board_led_level(bool on)
{
#if CONFIG_SLE_RL_LED_ACTIVE_LOW
    return on ? GPIO_LEVEL_LOW : GPIO_LEVEL_HIGH;
#else
    return on ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
#endif
}

bool sle_rl_board_led_init(void)
{
    if (uapi_pin_set_mode(CONFIG_SLE_RL_LED_PIN, PIN_MODE_0) != ERRCODE_SUCC) {
        return false;
    }
    if (uapi_gpio_set_dir(CONFIG_SLE_RL_LED_PIN, GPIO_DIRECTION_OUTPUT) != ERRCODE_SUCC) {
        return false;
    }
    return uapi_gpio_set_val(CONFIG_SLE_RL_LED_PIN, sle_rl_board_led_level(false)) == ERRCODE_SUCC;
}

bool sle_rl_board_led_set(bool on)
{
    return uapi_gpio_set_val(CONFIG_SLE_RL_LED_PIN, sle_rl_board_led_level(on)) == ERRCODE_SUCC;
}

bool sle_rl_board_led_toggle(void)
{
    return uapi_gpio_toggle(CONFIG_SLE_RL_LED_PIN) == ERRCODE_SUCC;
}

bool sle_rl_board_led_is_on(void)
{
    gpio_level_t level = uapi_gpio_get_output_val(CONFIG_SLE_RL_LED_PIN);
#if CONFIG_SLE_RL_LED_ACTIVE_LOW
    return level == GPIO_LEVEL_LOW;
#else
    return level == GPIO_LEVEL_HIGH;
#endif
}
