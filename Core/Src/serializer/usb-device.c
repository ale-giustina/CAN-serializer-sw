/**
 * @file usb-device.c
 * @brief Minimal USB full-speed device on top of HAL PCD.
 *
 * @details The HAL PCD driver moves packets; this file implements the
 * chapter 9 requests on EP0 and routes everything else to gs-usb.c.
 * On EP0 the HAL reports every packet, so multi-packet data stages are
 * continued here, 64 bytes at a time.
 */

#include "usb-device.h"

#include <stdbool.h>
#include <string.h>

#include "gs-usb-protocol.h"
#include "gs-usb.h"

#define EP0_MPS 64U
#define EP0_BUFFER_SIZE 128U

/* Packet memory layout (1 KB). The buffer descriptor table uses the first 64 bytes. */
#define PMA_EP0_OUT 0x40U
#define PMA_EP0_IN 0x80U
#define PMA_GS_IN 0xC0U
#define PMA_GS_OUT 0x100U

/* bmRequestType fields */
#define REQ_DIR_IN 0x80U
#define REQ_TYPE_MASK 0x60U
#define REQ_TYPE_STANDARD 0x00U
#define REQ_TYPE_VENDOR 0x40U
#define REQ_RECIPIENT_MASK 0x1FU
#define REQ_RECIPIENT_DEVICE 0x00U
#define REQ_RECIPIENT_INTERFACE 0x01U
#define REQ_RECIPIENT_ENDPOINT 0x02U

/* Standard requests */
#define REQ_GET_STATUS 0x00U
#define REQ_CLEAR_FEATURE 0x01U
#define REQ_SET_FEATURE 0x03U
#define REQ_SET_ADDRESS 0x05U
#define REQ_GET_DESCRIPTOR 0x06U
#define REQ_GET_CONFIGURATION 0x08U
#define REQ_SET_CONFIGURATION 0x09U
#define REQ_GET_INTERFACE 0x0AU
#define REQ_SET_INTERFACE 0x0BU

#define FEATURE_ENDPOINT_HALT 0x00U

#define DESC_DEVICE 0x01U
#define DESC_CONFIGURATION 0x02U
#define DESC_STRING 0x03U

#define STRING_LANGID 0x00U
#define STRING_MANUFACTURER 0x01U
#define STRING_PRODUCT 0x02U
#define STRING_SERIAL 0x03U
#define STRING_MS_OS 0xEEU

/* Microsoft OS 1.0 descriptors: let Windows bind WinUSB with no .inf file. */
#define MS_VENDOR_CODE 0x20U
#define MS_COMPAT_ID_INDEX 0x0004U

#define LO(x) ((uint8_t)((x) & 0xFFU))
#define HI(x) ((uint8_t)(((x) >> 8) & 0xFFU))

// clang-format off
static const uint8_t device_descriptor[] = {
    18U, DESC_DEVICE,
    0x00U, 0x02U,              /* bcdUSB 2.00 */
    0x00U, 0x00U, 0x00U,       /* class defined by the interface */
    EP0_MPS,
    LO(GS_USB_VID), HI(GS_USB_VID),
    LO(GS_USB_PID), HI(GS_USB_PID),
    0x00U, 0x01U,              /* bcdDevice 1.00 */
    STRING_MANUFACTURER, STRING_PRODUCT, STRING_SERIAL,
    1U,                        /* one configuration */
};

#define CONFIG_TOTAL_LENGTH (9U + 9U + 7U + 7U)

static const uint8_t config_descriptor[] = {
    9U, DESC_CONFIGURATION,
    LO(CONFIG_TOTAL_LENGTH), HI(CONFIG_TOTAL_LENGTH),
    1U,                        /* interfaces */
    1U,                        /* bConfigurationValue */
    0U,                        /* no string */
    0x80U,                     /* bus powered */
    50U,                       /* 100 mA */

    /* Interface 0: vendor specific, gs_usb */
    9U, 0x04U, 0U, 0U, 2U, 0xFFU, 0xFFU, 0xFFU, 0U,

    /* Bulk IN */
    7U, 0x05U, USB_EP_GS_IN, 0x02U, LO(USB_GS_EP_MPS), HI(USB_GS_EP_MPS), 0U,
    /* Bulk OUT */
    7U, 0x05U, USB_EP_GS_OUT, 0x02U, LO(USB_GS_EP_MPS), HI(USB_GS_EP_MPS), 0U,
};

static const uint8_t langid_descriptor[] = { 4U, DESC_STRING, 0x09U, 0x04U }; /* en-US */

static const uint8_t ms_os_string_descriptor[] = {
    18U, DESC_STRING,
    'M', 0U, 'S', 0U, 'F', 0U, 'T', 0U, '1', 0U, '0', 0U, '0', 0U,
    MS_VENDOR_CODE, 0U,
};

static const uint8_t ms_compat_id_descriptor[] = {
    40U, 0U, 0U, 0U,           /* dwLength */
    0x00U, 0x01U,              /* bcdVersion 1.00 */
    LO(MS_COMPAT_ID_INDEX), HI(MS_COMPAT_ID_INDEX),
    1U,                        /* one function section */
    0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U,                        /* interface 0 */
    1U,
    'W', 'I', 'N', 'U', 'S', 'B', 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U,
};
// clang-format on

_Static_assert(sizeof device_descriptor == 18U, "device descriptor size");
_Static_assert(sizeof config_descriptor == CONFIG_TOTAL_LENGTH, "configuration descriptor size");
_Static_assert(sizeof ms_compat_id_descriptor == 40U, "MS compat ID descriptor size");

static const char manufacturer_string[] = "AG";
static const char product_string[] = "CAN serializer";
static char serial_string[25]; /* 96-bit unique id as hex */

typedef enum {
    EP0_IDLE,
    EP0_DATA_IN,
    EP0_STATUS_OUT,
    EP0_DATA_OUT,
    EP0_STATUS_IN,
} ep0_state_t;

static PCD_HandleTypeDef *pcd;
static uint8_t configuration;

static usb_setup_t setup;
static ep0_state_t ep0_state;
static const uint8_t *ep0_tx_ptr;
static uint16_t ep0_tx_left;
static bool ep0_tx_zlp;
static uint16_t ep0_rx_len;
static uint16_t ep0_rx_got;
static uint8_t ep0_buffer[EP0_BUFFER_SIZE] __attribute__((aligned(4)));

/* ------------------------------------------------------------------------ */
/* EP0 transfers                                                            */
/* ------------------------------------------------------------------------ */

static void ep0_stall(void) {
    (void)HAL_PCD_EP_SetStall(pcd, 0x80U);
    (void)HAL_PCD_EP_SetStall(pcd, 0x00U);
    ep0_state = EP0_IDLE;
}

static void ep0_send_chunk(void) {
    const uint16_t n = (ep0_tx_left > EP0_MPS) ? EP0_MPS : ep0_tx_left;
    (void)HAL_PCD_EP_Transmit(pcd, 0x80U, (uint8_t *)ep0_tx_ptr, n);
    ep0_tx_ptr += n;
    ep0_tx_left = (uint16_t)(ep0_tx_left - n);
}

static void ep0_send(const uint8_t *data, uint16_t length) {
    if (length > setup.wLength) {
        length = setup.wLength;
    }
    /* A reply shorter than requested that ends on a full packet needs a ZLP. */
    ep0_tx_zlp = (length > 0U) && (length < setup.wLength) && ((length % EP0_MPS) == 0U);
    ep0_tx_ptr = data;
    ep0_tx_left = length;
    ep0_state = EP0_DATA_IN;
    ep0_send_chunk();
}

static void ep0_send_status(void) {
    ep0_state = EP0_STATUS_IN;
    (void)HAL_PCD_EP_Transmit(pcd, 0x80U, NULL, 0U);
}

static void ep0_data_in(void) {
    switch (ep0_state) {
        case EP0_DATA_IN:
            if (ep0_tx_left > 0U) {
                ep0_send_chunk();
            } else if (ep0_tx_zlp) {
                ep0_tx_zlp = false;
                (void)HAL_PCD_EP_Transmit(pcd, 0x80U, NULL, 0U);
            } else {
                ep0_state = EP0_STATUS_OUT;
                (void)HAL_PCD_EP_Receive(pcd, 0x00U, NULL, 0U);
            }
            break;
        case EP0_STATUS_IN:
            ep0_state = EP0_IDLE;
            break;
        default:
            break;
    }
}

static void ep0_data_out(void) {
    if (ep0_state != EP0_DATA_OUT) {
        return;
    }

    const uint32_t n = HAL_PCD_EP_GetRxCount(pcd, 0x00U);
    ep0_rx_got = (uint16_t)(ep0_rx_got + n);
    if ((ep0_rx_got < ep0_rx_len) && (n == EP0_MPS)) {
        const uint16_t left = (uint16_t)(ep0_rx_len - ep0_rx_got);
        (void)HAL_PCD_EP_Receive(pcd, 0x00U, &ep0_buffer[ep0_rx_got], (left > EP0_MPS) ? EP0_MPS : left);
        return;
    }

    if (gs_usb_control_out(&setup, ep0_buffer, ep0_rx_got)) {
        ep0_send_status();
    } else {
        ep0_stall();
    }
}

/* ------------------------------------------------------------------------ */
/* Standard requests                                                        */
/* ------------------------------------------------------------------------ */

static uint16_t build_string(const char *text) {
    const size_t len = strlen(text);
    const size_t max_chars = (EP0_BUFFER_SIZE - 2U) / 2U;
    const size_t chars = (len > max_chars) ? max_chars : len;

    ep0_buffer[0] = (uint8_t)(2U + (2U * chars));
    ep0_buffer[1] = DESC_STRING;
    for (size_t i = 0U; i < chars; i++) {
        ep0_buffer[2U + (2U * i)] = (uint8_t)text[i];
        ep0_buffer[3U + (2U * i)] = 0U;
    }
    return ep0_buffer[0];
}

static void get_descriptor(void) {
    const uint8_t type = HI(setup.wValue);
    const uint8_t index = LO(setup.wValue);

    switch (type) {
        case DESC_DEVICE:
            ep0_send(device_descriptor, sizeof device_descriptor);
            return;
        case DESC_CONFIGURATION:
            ep0_send(config_descriptor, sizeof config_descriptor);
            return;
        case DESC_STRING:
            switch (index) {
                case STRING_LANGID:
                    ep0_send(langid_descriptor, sizeof langid_descriptor);
                    return;
                case STRING_MANUFACTURER:
                    ep0_send(ep0_buffer, build_string(manufacturer_string));
                    return;
                case STRING_PRODUCT:
                    ep0_send(ep0_buffer, build_string(product_string));
                    return;
                case STRING_SERIAL:
                    ep0_send(ep0_buffer, build_string(serial_string));
                    return;
                case STRING_MS_OS:
                    ep0_send(ms_os_string_descriptor, sizeof ms_os_string_descriptor);
                    return;
                default:
                    break;
            }
            break;
        default:
            break; /* includes DEVICE_QUALIFIER: full-speed only */
    }
    ep0_stall();
}

static bool set_configuration(uint8_t value) {
    if (value > 1U) {
        return false;
    }
    if (configuration != 0U) {
        (void)HAL_PCD_EP_Close(pcd, USB_EP_GS_IN);
        (void)HAL_PCD_EP_Close(pcd, USB_EP_GS_OUT);
        configuration = 0U;
        gs_usb_on_configured(false);
    }
    if (value == 1U) {
        (void)HAL_PCD_EP_Open(pcd, USB_EP_GS_IN, USB_GS_EP_MPS, EP_TYPE_BULK);
        (void)HAL_PCD_EP_Open(pcd, USB_EP_GS_OUT, USB_GS_EP_MPS, EP_TYPE_BULK);
        configuration = 1U;
        gs_usb_on_configured(true);
    }
    return true;
}

static bool endpoint_is_ours(uint8_t ep_addr) {
    return (ep_addr == USB_EP_GS_IN) || (ep_addr == USB_EP_GS_OUT);
}

static bool endpoint_stalled(uint8_t ep_addr) {
    const uint8_t num = ep_addr & 0x7FU;
    return ((ep_addr & 0x80U) != 0U) ? (pcd->IN_ep[num].is_stall != 0U) : (pcd->OUT_ep[num].is_stall != 0U);
}

static void standard_request(void) {
    static uint8_t reply[2];
    const uint8_t recipient = setup.bmRequestType & REQ_RECIPIENT_MASK;
    const uint8_t ep_addr = LO(setup.wIndex);

    switch (setup.bRequest) {
        case REQ_GET_STATUS:
            reply[0] = 0U;
            reply[1] = 0U;
            if (recipient == REQ_RECIPIENT_ENDPOINT) {
                if (((ep_addr & 0x7FU) != 0U) && !endpoint_is_ours(ep_addr)) {
                    break;
                }
                reply[0] = ((ep_addr & 0x7FU) != 0U) && endpoint_stalled(ep_addr) ? 1U : 0U;
            }
            ep0_send(reply, 2U);
            return;

        case REQ_CLEAR_FEATURE:
        case REQ_SET_FEATURE:
            if (recipient == REQ_RECIPIENT_DEVICE) {
                ep0_send_status(); /* remote wakeup: accepted, not used */
                return;
            }
            if ((recipient == REQ_RECIPIENT_ENDPOINT) && (setup.wValue == FEATURE_ENDPOINT_HALT)) {
                if ((ep_addr & 0x7FU) == 0U) {
                    ep0_send_status();
                    return;
                }
                if (!endpoint_is_ours(ep_addr)) {
                    break;
                }
                if (setup.bRequest == REQ_SET_FEATURE) {
                    (void)HAL_PCD_EP_SetStall(pcd, ep_addr);
                } else {
                    (void)HAL_PCD_EP_ClrStall(pcd, ep_addr);
                    gs_usb_on_endpoint_reset(ep_addr);
                }
                ep0_send_status();
                return;
            }
            break;

        case REQ_SET_ADDRESS:
            /* The HAL applies the address once the status stage completes. */
            (void)HAL_PCD_SetAddress(pcd, (uint8_t)(setup.wValue & 0x7FU));
            ep0_send_status();
            return;

        case REQ_GET_DESCRIPTOR:
            get_descriptor();
            return;

        case REQ_GET_CONFIGURATION:
            reply[0] = configuration;
            ep0_send(reply, 1U);
            return;

        case REQ_SET_CONFIGURATION:
            if (set_configuration(LO(setup.wValue))) {
                ep0_send_status();
                return;
            }
            break;

        case REQ_GET_INTERFACE:
            if ((configuration != 0U) && (setup.wIndex == 0U)) {
                reply[0] = 0U;
                ep0_send(reply, 1U);
                return;
            }
            break;

        case REQ_SET_INTERFACE:
            if ((configuration != 0U) && (setup.wIndex == 0U) && (setup.wValue == 0U)) {
                ep0_send_status();
                return;
            }
            break;

        default:
            break;
    }
    ep0_stall();
}

/* Vendor requests whose data stage is IN, or that have no data stage. */
static void vendor_request(void) {
    const uint8_t recipient = setup.bmRequestType & REQ_RECIPIENT_MASK;

    if ((recipient != REQ_RECIPIENT_DEVICE) && (recipient != REQ_RECIPIENT_INTERFACE)) {
        ep0_stall();
        return;
    }

    if ((setup.bmRequestType & REQ_DIR_IN) != 0U) {
        if ((setup.bRequest == MS_VENDOR_CODE) && (setup.wIndex == MS_COMPAT_ID_INDEX)) {
            ep0_send(ms_compat_id_descriptor, sizeof ms_compat_id_descriptor);
            return;
        }
        const uint8_t *data = NULL;
        uint16_t length = 0U;
        if (gs_usb_control_in(&setup, &data, &length)) {
            ep0_send(data, length);
        } else {
            ep0_stall();
        }
        return;
    }

    if (gs_usb_control_out(&setup, NULL, 0U)) {
        ep0_send_status();
    } else {
        ep0_stall();
    }
}

/* ------------------------------------------------------------------------ */
/* HAL PCD callbacks (USB interrupt)                                        */
/* ------------------------------------------------------------------------ */

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
    const uint8_t *raw = (const uint8_t *)hpcd->Setup;
    setup.bmRequestType = raw[0];
    setup.bRequest = raw[1];
    setup.wValue = (uint16_t)(raw[2] | (raw[3] << 8));
    setup.wIndex = (uint16_t)(raw[4] | (raw[5] << 8));
    setup.wLength = (uint16_t)(raw[6] | (raw[7] << 8));

    ep0_state = EP0_IDLE;
    ep0_tx_left = 0U;
    ep0_tx_zlp = false;

    const uint8_t type = setup.bmRequestType & REQ_TYPE_MASK;
    const bool has_out_data = ((setup.bmRequestType & REQ_DIR_IN) == 0U) && (setup.wLength > 0U);

    if (has_out_data) {
        if ((type != REQ_TYPE_VENDOR) || (setup.wLength > EP0_BUFFER_SIZE)) {
            ep0_stall();
            return;
        }
        ep0_rx_len = setup.wLength;
        ep0_rx_got = 0U;
        ep0_state = EP0_DATA_OUT;
        (void)HAL_PCD_EP_Receive(hpcd, 0x00U, ep0_buffer, (ep0_rx_len > EP0_MPS) ? EP0_MPS : ep0_rx_len);
        return;
    }

    if (type == REQ_TYPE_STANDARD) {
        standard_request();
    } else if (type == REQ_TYPE_VENDOR) {
        vendor_request();
    } else {
        ep0_stall();
    }
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    (void)hpcd;
    if (epnum == 0U) {
        ep0_data_in();
    } else if (epnum == (USB_EP_GS_IN & 0x7FU)) {
        gs_usb_on_in_complete();
    }
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
    if (epnum == 0U) {
        ep0_data_out();
    } else if (epnum == USB_EP_GS_OUT) {
        gs_usb_on_out_complete(HAL_PCD_EP_GetRxCount(hpcd, epnum));
    }
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
    (void)HAL_PCD_EP_Open(hpcd, 0x00U, EP0_MPS, EP_TYPE_CTRL);
    (void)HAL_PCD_EP_Open(hpcd, 0x80U, EP0_MPS, EP_TYPE_CTRL);
    ep0_state = EP0_IDLE;
    if (configuration != 0U) {
        configuration = 0U;
        gs_usb_on_configured(false);
    }
}

/* ------------------------------------------------------------------------ */
/* Init                                                                     */
/* ------------------------------------------------------------------------ */

static void build_serial_string(void) {
    static const char hex[] = "0123456789ABCDEF";
    const uint32_t uid[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
    size_t pos = 0U;
    for (size_t w = 0U; w < 3U; w++) {
        for (int shift = 28; shift >= 0; shift -= 4) {
            serial_string[pos++] = hex[(uid[w] >> shift) & 0xFU];
        }
    }
    serial_string[pos] = '\0';
}

/* HSI48 alone is not accurate enough for USB: trim it continuously on the host's SOF. */
static void enable_crs(void) {
    RCC_CRSInitTypeDef crs = {
        .Prescaler = RCC_CRS_SYNC_DIV1,
        .Source = RCC_CRS_SYNC_SOURCE_USB,
        .Polarity = RCC_CRS_SYNC_POLARITY_RISING,
        .ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT,
        .ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT,
        .HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT,
    };
    __HAL_RCC_CRS_CLK_ENABLE();
    HAL_RCCEx_CRSConfig(&crs);
}

void usb_device_init(PCD_HandleTypeDef *hpcd) {
    pcd = hpcd;
    configuration = 0U;
    ep0_state = EP0_IDLE;
    build_serial_string();
    enable_crs();

    (void)HAL_PCDEx_PMAConfig(hpcd, 0x00U, PCD_SNG_BUF, PMA_EP0_OUT);
    (void)HAL_PCDEx_PMAConfig(hpcd, 0x80U, PCD_SNG_BUF, PMA_EP0_IN);
    (void)HAL_PCDEx_PMAConfig(hpcd, USB_EP_GS_IN, PCD_SNG_BUF, PMA_GS_IN);
    (void)HAL_PCDEx_PMAConfig(hpcd, USB_EP_GS_OUT, PCD_SNG_BUF, PMA_GS_OUT);

    (void)HAL_PCD_Start(hpcd);
}
