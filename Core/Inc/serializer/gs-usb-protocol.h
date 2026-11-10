/**
 * @file gs-usb-protocol.h
 * @brief Wire format of the gs_usb protocol (candleLight compatible).
 *
 * @details Mirrors the definitions used by the Linux gs_usb driver
 * (drivers/net/can/usb/gs_usb.c). Every field is little endian, which is
 * also the native byte order of the Cortex-M4, so structs are sent as-is.
 */

#ifndef GS_USB_PROTOCOL_H
#define GS_USB_PROTOCOL_H

#include <stdint.h>

/* USB identity. 1d50:606f is the candleLight id that the Linux gs_usb driver binds to. */
#define GS_USB_VID 0x1D50U
#define GS_USB_PID 0x606FU

/* Vendor control requests (bRequest), sent to the interface. */
enum gs_usb_breq {
    GS_USB_BREQ_HOST_FORMAT = 0,
    GS_USB_BREQ_BITTIMING = 1,
    GS_USB_BREQ_MODE = 2,
    GS_USB_BREQ_BERR = 3,
    GS_USB_BREQ_BT_CONST = 4,
    GS_USB_BREQ_DEVICE_CONFIG = 5,
    GS_USB_BREQ_TIMESTAMP = 6,
    GS_USB_BREQ_IDENTIFY = 7,
    GS_USB_BREQ_GET_USER_ID = 8,
    GS_USB_BREQ_SET_USER_ID = 9,
    GS_USB_BREQ_DATA_BITTIMING = 10,
    GS_USB_BREQ_BT_CONST_EXT = 11,
    GS_USB_BREQ_SET_TERMINATION = 12,
    GS_USB_BREQ_GET_TERMINATION = 13,
    GS_USB_BREQ_GET_STATE = 14,
};

/* gs_device_mode.mode */
#define GS_CAN_MODE_RESET 0U
#define GS_CAN_MODE_START 1U

/* Feature bits (gs_device_bt_const.feature) and the matching mode flags (gs_device_mode.flags). */
#define GS_CAN_FEATURE_LISTEN_ONLY (1UL << 0)
#define GS_CAN_FEATURE_LOOP_BACK (1UL << 1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE (1UL << 2)
#define GS_CAN_FEATURE_ONE_SHOT (1UL << 3)
#define GS_CAN_FEATURE_HW_TIMESTAMP (1UL << 4)
#define GS_CAN_FEATURE_IDENTIFY (1UL << 5)
#define GS_CAN_FEATURE_USER_ID (1UL << 6)
#define GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE (1UL << 7)
#define GS_CAN_FEATURE_FD (1UL << 8)
#define GS_CAN_FEATURE_REQ_USB_QUIRK_LPC546XX (1UL << 9)
#define GS_CAN_FEATURE_BT_CONST_EXT (1UL << 10)
#define GS_CAN_FEATURE_TERMINATION (1UL << 11)
#define GS_CAN_FEATURE_BERR_REPORTING (1UL << 12)
#define GS_CAN_FEATURE_GET_STATE (1UL << 13)

#define GS_CAN_MODE_NORMAL 0UL
#define GS_CAN_MODE_LISTEN_ONLY GS_CAN_FEATURE_LISTEN_ONLY
#define GS_CAN_MODE_LOOP_BACK GS_CAN_FEATURE_LOOP_BACK
#define GS_CAN_MODE_TRIPLE_SAMPLE GS_CAN_FEATURE_TRIPLE_SAMPLE
#define GS_CAN_MODE_ONE_SHOT GS_CAN_FEATURE_ONE_SHOT
#define GS_CAN_MODE_HW_TIMESTAMP GS_CAN_FEATURE_HW_TIMESTAMP
#define GS_CAN_MODE_PAD_PKTS_TO_MAX_PKT_SIZE GS_CAN_FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE
#define GS_CAN_MODE_FD GS_CAN_FEATURE_FD
#define GS_CAN_MODE_BERR_REPORTING GS_CAN_FEATURE_BERR_REPORTING

/* gs_host_frame.flags */
#define GS_CAN_FLAG_OVERFLOW (1U << 0)
#define GS_CAN_FLAG_FD (1U << 1)
#define GS_CAN_FLAG_BRS (1U << 2)
#define GS_CAN_FLAG_ESI (1U << 3)

/* echo_id of a frame received from the bus (anything else is a TX echo). */
#define GS_HOST_FRAME_ECHO_ID_RX 0xFFFFFFFFUL

/* Linux SocketCAN id layout (linux/can.h). */
#define CAN_EFF_FLAG 0x80000000UL
#define CAN_RTR_FLAG 0x40000000UL
#define CAN_ERR_FLAG 0x20000000UL
#define CAN_SFF_MASK 0x000007FFUL
#define CAN_EFF_MASK 0x1FFFFFFFUL

/* Error frame layout (linux/can/error.h). */
#define CAN_ERR_DLC 8U
#define CAN_ERR_CRTL 0x00000004UL
#define CAN_ERR_BUSOFF 0x00000040UL
#define CAN_ERR_RESTARTED 0x00000100UL
#define CAN_ERR_CNT 0x00000200UL

#define CAN_ERR_CRTL_RX_OVERFLOW 0x01U
#define CAN_ERR_CRTL_RX_WARNING 0x04U
#define CAN_ERR_CRTL_TX_WARNING 0x08U
#define CAN_ERR_CRTL_RX_PASSIVE 0x10U
#define CAN_ERR_CRTL_TX_PASSIVE 0x20U
#define CAN_ERR_CRTL_ACTIVE 0x40U

/* gs_device_state.state */
enum gs_can_state {
    GS_CAN_STATE_ERROR_ACTIVE = 0,
    GS_CAN_STATE_ERROR_WARNING = 1,
    GS_CAN_STATE_ERROR_PASSIVE = 2,
    GS_CAN_STATE_BUS_OFF = 3,
    GS_CAN_STATE_STOPPED = 4,
    GS_CAN_STATE_SLEEPING = 5,
};

struct gs_host_config {
    uint32_t byte_order;
} __attribute__((packed));

struct gs_device_config {
    uint8_t reserved1;
    uint8_t reserved2;
    uint8_t reserved3;
    uint8_t icount; /* number of CAN channels minus one */
    uint32_t sw_version;
    uint32_t hw_version;
} __attribute__((packed));

struct gs_device_mode {
    uint32_t mode;
    uint32_t flags;
} __attribute__((packed));

struct gs_device_state {
    uint32_t state;
    uint32_t rxerr;
    uint32_t txerr;
} __attribute__((packed));

struct gs_device_bittiming {
    uint32_t prop_seg;
    uint32_t phase_seg1;
    uint32_t phase_seg2;
    uint32_t sjw;
    uint32_t brp;
} __attribute__((packed));

struct gs_identify_mode {
    uint32_t mode;
} __attribute__((packed));

struct gs_device_bt_const {
    uint32_t feature;
    uint32_t fclk_can;
    uint32_t tseg1_min;
    uint32_t tseg1_max;
    uint32_t tseg2_min;
    uint32_t tseg2_max;
    uint32_t sjw_max;
    uint32_t brp_min;
    uint32_t brp_max;
    uint32_t brp_inc;
} __attribute__((packed));

struct gs_device_bt_const_extended {
    uint32_t feature;
    uint32_t fclk_can;
    uint32_t tseg1_min;
    uint32_t tseg1_max;
    uint32_t tseg2_min;
    uint32_t tseg2_max;
    uint32_t sjw_max;
    uint32_t brp_min;
    uint32_t brp_max;
    uint32_t brp_inc;
    uint32_t dtseg1_min;
    uint32_t dtseg1_max;
    uint32_t dtseg2_min;
    uint32_t dtseg2_max;
    uint32_t dsjw_max;
    uint32_t dbrp_min;
    uint32_t dbrp_max;
    uint32_t dbrp_inc;
} __attribute__((packed));

#define GS_HOST_FRAME_HEADER_SIZE 12U
#define GS_HOST_FRAME_CLASSIC_DATA 8U
#define GS_HOST_FRAME_FD_DATA 64U
#define GS_HOST_FRAME_TIMESTAMP_SIZE 4U

/**
 * @brief One CAN frame on the bulk endpoints.
 *
 * @details The payload area holds 8 data bytes for classic frames or 64 for
 * FD frames, optionally followed by a 32-bit microsecond timestamp. The
 * transfer length tells which layout is in use.
 */
struct gs_host_frame {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t channel;
    uint8_t flags;
    uint8_t reserved;
    uint8_t payload[GS_HOST_FRAME_FD_DATA + GS_HOST_FRAME_TIMESTAMP_SIZE];
} __attribute__((packed, aligned(4)));

#define GS_HOST_FRAME_MAX_SIZE ((uint32_t)sizeof(struct gs_host_frame))

_Static_assert(sizeof(struct gs_device_config) == 12U, "gs_device_config layout");
_Static_assert(sizeof(struct gs_device_bittiming) == 20U, "gs_device_bittiming layout");
_Static_assert(sizeof(struct gs_device_bt_const) == 40U, "gs_device_bt_const layout");
_Static_assert(sizeof(struct gs_device_bt_const_extended) == 72U, "gs_device_bt_const_extended layout");
_Static_assert(sizeof(struct gs_host_frame) == 80U, "gs_host_frame layout");

#endif /* GS_USB_PROTOCOL_H */
