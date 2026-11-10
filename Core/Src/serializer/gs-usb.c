/**
 * @file gs-usb.c
 * @brief gs_usb device class: makes the board a 3-channel candleLight-style adapter.
 *
 * @details
 * Bulk IN (0x81): one frame per transfer, taken from a 256-entry queue that
 * the FDCAN interrupts fill with received frames, TX echoes and error frames.
 *
 * Bulk OUT (0x02): one frame per transfer, pushed to the TX queue of the
 * channel it names. The endpoint is only re-armed while every channel queue
 * has room, so a busy bus slows the host down (USB NAK) instead of losing frames.
 */

#include "gs-usb.h"

#include <string.h>

#include "can-bridge.h"
#include "gs-usb-protocol.h"
#include "leds.h"
#include "timestamp.h"
#include "usb.h"

#define IN_QUEUE_LEN 256U
#define GS_SW_VERSION 2U /* > 1 lets the Linux driver expose identify */
#define GS_HW_VERSION 1U

#define GS_FEATURES                                                                    \
    (GS_CAN_FEATURE_LISTEN_ONLY | GS_CAN_FEATURE_LOOP_BACK | GS_CAN_FEATURE_ONE_SHOT | \
     GS_CAN_FEATURE_HW_TIMESTAMP | GS_CAN_FEATURE_IDENTIFY | GS_CAN_FEATURE_FD |       \
     GS_CAN_FEATURE_BT_CONST_EXT | GS_CAN_FEATURE_GET_STATE)

_Static_assert((IN_QUEUE_LEN & (IN_QUEUE_LEN - 1U)) == 0U, "IN_QUEUE_LEN must be a power of two");
_Static_assert((GS_FEATURES & ~(CAN_BRIDGE_SUPPORTED_MODES | GS_CAN_FEATURE_IDENTIFY | GS_CAN_FEATURE_BT_CONST_EXT |
                                GS_CAN_FEATURE_GET_STATE)) == 0U,
               "every advertised mode must be implemented by the bridge");

typedef enum {
    MODE_REQUEST_NONE = 0,
    MODE_REQUEST_START,
    MODE_REQUEST_RESET,
} mode_request_t;

static queued_frame_t in_slots[IN_QUEUE_LEN];
static frame_queue_t in_queue;

static volatile bool configured;
static volatile bool in_busy;
static volatile bool out_armed;
static volatile bool out_paused;
static struct gs_host_frame out_frame;

/* Written by control requests (USB interrupt), applied by gs_usb_poll(). */
static can_bridge_config_t channel_config[CAN_BRIDGE_CHANNELS];
static volatile uint8_t mode_request[CAN_BRIDGE_CHANNELS];
static volatile bool accepting[CAN_BRIDGE_CHANNELS]; /* host frames for this channel are queued */

static union {
    struct gs_device_config device_config;
    struct gs_device_bt_const bt_const;
    struct gs_device_bt_const_extended bt_const_ext;
    struct gs_device_state state;
    uint32_t timestamp;
} reply;

/* ------------------------------------------------------------------------ */
/* Bulk pipes (USB interrupt)                                               */
/* ------------------------------------------------------------------------ */

static void pump_in(void) {
    if (!configured || in_busy) {
        return;
    }
    queued_frame_t *q = frame_queue_peek(&in_queue);
    if (q == NULL) {
        return;
    }
    in_busy = true;
    (void)HAL_PCD_EP_Transmit(&hpcd_USB_FS, USB_EP_GS_IN, (uint8_t *)&q->frame, q->size);
}

static bool tx_queues_have_space(void) {
    for (uint8_t ch = 0U; ch < CAN_BRIDGE_CHANNELS; ch++) {
        if (frame_queue_space(can_bridge_tx_queue(ch)) == 0U) {
            return false;
        }
    }
    return true;
}

static void arm_out(void) {
    if (!configured || out_armed) {
        return;
    }
    /* Mark paused before checking, so a queue drained in between still wakes us. */
    out_paused = true;
    if (!tx_queues_have_space()) {
        return;
    }
    out_paused = false;
    out_armed = true;
    (void)HAL_PCD_EP_Receive(&hpcd_USB_FS, USB_EP_GS_OUT, (uint8_t *)&out_frame, sizeof out_frame);
}

void gs_usb_on_in_complete(void) {
    if (in_busy) {
        in_busy = false;
        frame_queue_pop(&in_queue);
    }
    pump_in();
}

void gs_usb_on_out_complete(uint32_t length) {
    out_armed = false;

    if (length >= GS_HOST_FRAME_HEADER_SIZE) {
        const uint8_t ch = out_frame.channel;
        frame_queue_t *txq = can_bridge_tx_queue(ch);
        if ((txq != NULL) && accepting[ch]) {
            queued_frame_t *q = frame_queue_reserve(txq);
            if (q != NULL) {
                q->frame = out_frame;
                q->size = length;
                frame_queue_commit(txq);
                can_bridge_kick_tx(ch);
            }
        }
    }
    arm_out();
}

void gs_usb_on_endpoint_reset(uint8_t ep_addr) {
    if (ep_addr == USB_EP_GS_IN) {
        in_busy = false; /* the frame being sent stays queued and is sent again */
        pump_in();
    } else if (ep_addr == USB_EP_GS_OUT) {
        out_armed = false;
        arm_out();
    }
}

void gs_usb_on_configured(bool is_configured) {
    configured = is_configured;
    in_busy = false;
    out_armed = false;
    out_paused = false;
    frame_queue_flush(&in_queue);

    if (is_configured) {
        arm_out();
        pump_in();
        return;
    }
    for (uint8_t ch = 0U; ch < CAN_BRIDGE_CHANNELS; ch++) {
        accepting[ch] = false;
        mode_request[ch] = MODE_REQUEST_RESET;
    }
}

void gs_usb_irq(void) {
    arm_out();
    pump_in();
}

/* ------------------------------------------------------------------------ */
/* Producers in other contexts                                              */
/* ------------------------------------------------------------------------ */

queued_frame_t *gs_usb_in_reserve(void) {
    return frame_queue_reserve(&in_queue);
}

void gs_usb_in_commit(void) {
    frame_queue_commit(&in_queue);
    if (configured && !in_busy) {
        NVIC_SetPendingIRQ(USB_LP_IRQn);
    }
}

void gs_usb_notify_tx_space(void) {
    if (out_paused) {
        NVIC_SetPendingIRQ(USB_LP_IRQn);
    }
}

/* ------------------------------------------------------------------------ */
/* Control requests (USB interrupt)                                         */
/* ------------------------------------------------------------------------ */

static struct gs_device_bt_const nominal_const(void) {
    const struct gs_device_bt_const bt = {
        .feature = GS_FEATURES,
        .fclk_can = CAN_BRIDGE_FDCAN_CLOCK_HZ,
        .tseg1_min = CAN_BRIDGE_NTSEG1_MIN,
        .tseg1_max = CAN_BRIDGE_NTSEG1_MAX,
        .tseg2_min = CAN_BRIDGE_NTSEG2_MIN,
        .tseg2_max = CAN_BRIDGE_NTSEG2_MAX,
        .sjw_max = CAN_BRIDGE_NSJW_MAX,
        .brp_min = 1U,
        .brp_max = CAN_BRIDGE_NBRP_MAX,
        .brp_inc = 1U,
    };
    return bt;
}

bool gs_usb_control_in(const usb_setup_t *setup, const uint8_t **data, uint16_t *length) {
    const uint16_t ch = setup->wValue;

    switch (setup->bRequest) {
        case GS_USB_BREQ_DEVICE_CONFIG:
            memset(&reply.device_config, 0, sizeof reply.device_config);
            reply.device_config.icount = CAN_BRIDGE_CHANNELS - 1U;
            reply.device_config.sw_version = GS_SW_VERSION;
            reply.device_config.hw_version = GS_HW_VERSION;
            *length = sizeof reply.device_config;
            break;

        case GS_USB_BREQ_BT_CONST:
            if (ch >= CAN_BRIDGE_CHANNELS) {
                return false;
            }
            reply.bt_const = nominal_const();
            *length = sizeof reply.bt_const;
            break;

        case GS_USB_BREQ_BT_CONST_EXT: {
            if (ch >= CAN_BRIDGE_CHANNELS) {
                return false;
            }
            /* The extended struct starts with the same fields as the nominal one. */
            const struct gs_device_bt_const nominal = nominal_const();
            struct gs_device_bt_const_extended *bt = &reply.bt_const_ext;
            memcpy(bt, &nominal, sizeof nominal);
            bt->dtseg1_min = CAN_BRIDGE_DTSEG1_MIN;
            bt->dtseg1_max = CAN_BRIDGE_DTSEG1_MAX;
            bt->dtseg2_min = CAN_BRIDGE_DTSEG2_MIN;
            bt->dtseg2_max = CAN_BRIDGE_DTSEG2_MAX;
            bt->dsjw_max = CAN_BRIDGE_DSJW_MAX;
            bt->dbrp_min = 1U;
            bt->dbrp_max = CAN_BRIDGE_DBRP_MAX;
            bt->dbrp_inc = 1U;
            *length = sizeof *bt;
            break;
        }

        case GS_USB_BREQ_TIMESTAMP:
            reply.timestamp = timestamp_us();
            *length = sizeof reply.timestamp;
            break;

        case GS_USB_BREQ_GET_STATE:
            if (ch >= CAN_BRIDGE_CHANNELS) {
                return false;
            }
            can_bridge_get_state((uint8_t)ch, &reply.state);
            *length = sizeof reply.state;
            break;

        default:
            return false;
    }

    *data = (const uint8_t *)&reply;
    return true;
}

static bool set_mode(uint8_t ch, const struct gs_device_mode *mode) {
    if (mode->mode == GS_CAN_MODE_RESET) {
        accepting[ch] = false;
        mode_request[ch] = MODE_REQUEST_RESET;
        return true;
    }
    if (mode->mode != GS_CAN_MODE_START) {
        return false;
    }
    if ((mode->flags & ~CAN_BRIDGE_SUPPORTED_MODES) != 0U) {
        return false;
    }
    channel_config[ch].flags = mode->flags;
    accepting[ch] = true;
    mode_request[ch] = MODE_REQUEST_START;
    return true;
}

bool gs_usb_control_out(const usb_setup_t *setup, const uint8_t *data, uint16_t length) {
    const uint16_t ch = setup->wValue;

    switch (setup->bRequest) {
        case GS_USB_BREQ_HOST_FORMAT:
            /* Host and device are both little endian; nothing to configure. */
            return length == sizeof(struct gs_host_config);

        case GS_USB_BREQ_BITTIMING:
        case GS_USB_BREQ_DATA_BITTIMING: {
            struct gs_device_bittiming bt;
            if ((ch >= CAN_BRIDGE_CHANNELS) || (length != sizeof bt)) {
                return false;
            }
            memcpy(&bt, data, sizeof bt);
            if (setup->bRequest == GS_USB_BREQ_BITTIMING) {
                if (!can_bridge_nominal_timing_valid(&bt)) {
                    return false;
                }
                channel_config[ch].nominal = bt;
            } else {
                if (!can_bridge_data_timing_valid(&bt)) {
                    return false;
                }
                channel_config[ch].data = bt;
            }
            return true;
        }

        case GS_USB_BREQ_MODE: {
            struct gs_device_mode mode;
            if ((ch >= CAN_BRIDGE_CHANNELS) || (length != sizeof mode)) {
                return false;
            }
            memcpy(&mode, data, sizeof mode);
            return set_mode((uint8_t)ch, &mode);
        }

        case GS_USB_BREQ_IDENTIFY: {
            struct gs_identify_mode identify;
            if ((ch >= CAN_BRIDGE_CHANNELS) || (length != sizeof identify)) {
                return false;
            }
            memcpy(&identify, data, sizeof identify);
            leds_set_identify((uint8_t)ch, identify.mode != 0U);
            return true;
        }

        default:
            return false;
    }
}

/* ------------------------------------------------------------------------ */
/* Main loop                                                                */
/* ------------------------------------------------------------------------ */

void gs_usb_init(void) {
    frame_queue_init(&in_queue, in_slots, IN_QUEUE_LEN);
    for (uint8_t ch = 0U; ch < CAN_BRIDGE_CHANNELS; ch++) {
        can_bridge_default_config(ch, &channel_config[ch]);
        mode_request[ch] = MODE_REQUEST_NONE;
        accepting[ch] = false;
    }
}

void gs_usb_poll(void) {
    for (uint8_t ch = 0U; ch < CAN_BRIDGE_CHANNELS; ch++) {
        if (mode_request[ch] == MODE_REQUEST_NONE) {
            continue;
        }

        HAL_NVIC_DisableIRQ(USB_LP_IRQn);
        const uint8_t request = mode_request[ch];
        const can_bridge_config_t config = channel_config[ch];
        mode_request[ch] = MODE_REQUEST_NONE;
        HAL_NVIC_EnableIRQ(USB_LP_IRQn);

        if (request == MODE_REQUEST_START) {
            if (can_bridge_start(ch, &config)) {
                continue;
            }
            accepting[ch] = false; /* start failed: leave the channel stopped */
        } else {
            leds_set_identify(ch, false);
        }
        can_bridge_stop(ch);
    }
}
