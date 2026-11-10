/**
 * @file serializer.c
 * @brief Entry points called from main.c.
 *
 * @details CAN traffic is handled entirely in the FDCAN and USB interrupts.
 * The main loop only applies channel start/stop requests from the host and
 * drives the LEDs.
 */

#include "serializer.h"

#include "can-bridge.h"
#include "gs-usb.h"
#include "leds.h"
#include "timestamp.h"
#include "usb-device.h"
#include "usb.h"

void serializer_init(void) {
    timestamp_init();
    leds_init();
    can_bridge_init();
    gs_usb_init();
    usb_device_init(&hpcd_USB_FS);
}

void serializer_loop(void) {
    gs_usb_poll();
    leds_update(timestamp_us());
}
