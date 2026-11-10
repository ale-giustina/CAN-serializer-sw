/**
 * @file gs-usb.h
 * @brief gs_usb device class: makes the board a 3-channel candleLight-style adapter.
 *
 * @details
 * Execution contexts:
 * - gs_usb_on_*() and gs_usb_control_*(): USB interrupt (called by usb-device).
 * - gs_usb_irq(): end of USB_LP_IRQHandler; also runs when another context
 *   pends the USB interrupt to restart the bulk pipes.
 * - gs_usb_in_reserve() / gs_usb_in_commit() / gs_usb_notify_tx_space():
 *   FDCAN interrupts.
 * - gs_usb_poll(): main loop; applies channel start/stop requests, which
 *   cannot run in an interrupt because they reinitialise an FDCAN.
 */

#ifndef GS_USB_H
#define GS_USB_H

#include <stdbool.h>
#include <stdint.h>

#include "frame-queue.h"
#include "usb-device.h"

void gs_usb_init(void);
void gs_usb_poll(void);
void gs_usb_irq(void);

/* Device-to-host queue, produced by the FDCAN interrupts. */
queued_frame_t *gs_usb_in_reserve(void);
void gs_usb_in_commit(void);

/* A channel TX queue gained space: resume reading from the host if paused. */
void gs_usb_notify_tx_space(void);

/* USB device callbacks. */
void gs_usb_on_configured(bool configured);
bool gs_usb_control_in(const usb_setup_t *setup, const uint8_t **data, uint16_t *length);
bool gs_usb_control_out(const usb_setup_t *setup, const uint8_t *data, uint16_t length);
void gs_usb_on_in_complete(void);
void gs_usb_on_out_complete(uint32_t length);
void gs_usb_on_endpoint_reset(uint8_t ep_addr);

#endif /* GS_USB_H */
