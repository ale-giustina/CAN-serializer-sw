/**
 * @file usb-device.h
 * @brief Minimal USB full-speed device on top of HAL PCD.
 *
 * @details Handles enumeration (descriptors, address, configuration),
 * Microsoft OS 1.0 descriptors so Windows binds WinUSB without a driver, and
 * forwards vendor requests and the two bulk endpoints to the gs_usb layer.
 * Everything runs in the USB interrupt.
 */

#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#include <stdint.h>

#include "main.h"

#define USB_EP_GS_IN 0x81U
#define USB_EP_GS_OUT 0x02U
#define USB_GS_EP_MPS 64U

typedef struct {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_t;

/** Configures the packet memory and connects to the host. Call after MX_USB_PCD_Init(). */
void usb_device_init(PCD_HandleTypeDef *hpcd);

#endif /* USB_DEVICE_H */
