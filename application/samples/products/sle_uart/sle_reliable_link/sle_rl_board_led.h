/**
 * BearPi-Pico H3863 on-board LED adapter for the reliable-link sample.
 */
#ifndef SLE_RL_BOARD_LED_H
#define SLE_RL_BOARD_LED_H

#include <stdbool.h>

#ifndef CONFIG_SLE_RL_LED_PIN
#define CONFIG_SLE_RL_LED_PIN 2
#endif

#ifndef CONFIG_SLE_RL_LED_ACTIVE_LOW
/* BearPi-Pico H3863: GPIO2 high = LED on（实测），因此默认 active high */
#define CONFIG_SLE_RL_LED_ACTIVE_LOW 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool sle_rl_board_led_init(void);
bool sle_rl_board_led_set(bool on);
bool sle_rl_board_led_toggle(void);
bool sle_rl_board_led_is_on(void);

#ifdef __cplusplus
}
#endif

#endif
