/**
 * @file leds.h
 * @brief RX/TX activity LEDs, one pair per CAN channel.
 *
 * @details
 * - RX / TX LED flashes for each frame received / transmitted.
 * - Both LEDs solid: the channel is bus-off.
 * - Both LEDs blinking together: identify requested by the host.
 *
 * leds_activity(), leds_set_fault() and leds_set_identify() are safe to call
 * from interrupts; leds_update() runs in the main loop.
 */

#ifndef LEDS_H
#define LEDS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LED_RX = 0,
    LED_TX = 1,
} led_kind_t;

void leds_init(void);
void leds_activity(uint8_t channel, led_kind_t kind);
void leds_set_fault(uint8_t channel, bool fault);
void leds_set_identify(uint8_t channel, bool identify);
void leds_update(uint32_t now_us);

#endif /* LEDS_H */
