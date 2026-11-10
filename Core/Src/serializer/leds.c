/**
 * @file leds.c
 * @brief RX/TX activity LEDs, one pair per CAN channel.
 */

#include "leds.h"

#include "can-bridge.h"
#include "main.h"

#define LED_FLASH_US 20000U
#define LED_IDENTIFY_HALF_PERIOD_US 250000U
#define LED_BOOT_STEP_MS 60U

/* The LEDs are driven active high. */
#define LED_ON GPIO_PIN_SET
#define LED_OFF GPIO_PIN_RESET

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} led_pin_t;

static const led_pin_t led_pins[CAN_BRIDGE_CHANNELS][2] = {
    { { CAN1_LED_RX_GPIO_Port, CAN1_LED_RX_Pin }, { CAN1_LED_TX_GPIO_Port, CAN1_LED_TX_Pin } },
    { { CAN2_LED_RX_GPIO_Port, CAN2_LED_RX_Pin }, { CAN2_LED_TX_GPIO_Port, CAN2_LED_TX_Pin } },
    { { CAN3_LED_RX_GPIO_Port, CAN3_LED_RX_Pin }, { CAN3_LED_TX_GPIO_Port, CAN3_LED_TX_Pin } },
};

static volatile bool activity[CAN_BRIDGE_CHANNELS][2];
static volatile bool fault[CAN_BRIDGE_CHANNELS];
static volatile bool identify[CAN_BRIDGE_CHANNELS];
static bool lit[CAN_BRIDGE_CHANNELS][2];
static uint32_t lit_since[CAN_BRIDGE_CHANNELS][2];

static void led_write(uint8_t channel, led_kind_t kind, bool on) {
    const led_pin_t *led = &led_pins[channel][kind];
    HAL_GPIO_WritePin(led->port, led->pin, on ? LED_ON : LED_OFF);
}

void leds_init(void) {
    /* Short sweep at power-up to show every LED works. */
    for (uint8_t ch = 0U; ch < CAN_BRIDGE_CHANNELS; ch++) {
        for (uint8_t kind = 0U; kind < 2U; kind++) {
            led_write(ch, (led_kind_t)kind, true);
            HAL_Delay(LED_BOOT_STEP_MS);
            led_write(ch, (led_kind_t)kind, false);
        }
    }
}

void leds_activity(uint8_t channel, led_kind_t kind) {
    if (channel < CAN_BRIDGE_CHANNELS) {
        activity[channel][kind] = true;
    }
}

void leds_set_fault(uint8_t channel, bool is_fault) {
    if (channel < CAN_BRIDGE_CHANNELS) {
        fault[channel] = is_fault;
    }
}

void leds_set_identify(uint8_t channel, bool on) {
    if (channel < CAN_BRIDGE_CHANNELS) {
        identify[channel] = on;
    }
}

void leds_update(uint32_t now_us) {
    const bool blink_phase = ((now_us / LED_IDENTIFY_HALF_PERIOD_US) & 1U) != 0U;

    for (uint8_t ch = 0U; ch < CAN_BRIDGE_CHANNELS; ch++) {
        for (uint8_t kind = 0U; kind < 2U; kind++) {
            if (activity[ch][kind]) {
                activity[ch][kind] = false;
                lit[ch][kind] = true;
                lit_since[ch][kind] = now_us;
            } else if (lit[ch][kind] && ((now_us - lit_since[ch][kind]) >= LED_FLASH_US)) {
                lit[ch][kind] = false;
            }

            const bool on = identify[ch] ? blink_phase : (fault[ch] || lit[ch][kind]);
            led_write(ch, (led_kind_t)kind, on);
        }
    }
}
