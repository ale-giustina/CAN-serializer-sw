/**
 * @file can-bridge.c
 * @brief Runs the three FDCAN channels on behalf of the gs_usb host.
 *
 * @details
 * RX: both RX FIFOs (standard ids -> FIFO0, extended ids -> FIFO1, no
 * filters) are drained in the interrupt straight into the gs_usb IN queue.
 * The G4 FIFOs are only 3 messages deep, so draining is never deferred.
 *
 * TX: frames from the host wait in a per-channel software queue. The
 * channel interrupt moves them into the 3-entry hardware TX FIFO. Each frame
 * in flight keeps a copy in a slot whose index is sent as the message
 * marker. When the TX event FIFO reports the marker (or, in one-shot mode, the
 * transmission is cancelled), the copy goes back to the host as the echo that
 * frees the host's TX context.
 *
 * Errors: state changes (warning, passive, bus-off) become SocketCAN error
 * frames. Bus-off recovers automatically, as on candleLight adapters.
 */

#include "can-bridge.h"

#include <string.h>

#include "fdcan.h"
#include "gs-usb.h"
#include "leds.h"
#include "timestamp.h"

#define TX_QUEUE_LEN 16U /* host frames waiting for the hardware, per channel */
#define TX_SLOTS 8U      /* frames handed to the hardware but not echoed yet */
#define HW_TX_BUFFERS 3U /* G4 FDCAN TX FIFO depth */
#define NO_SLOT 0xFFU

/* TDC is only needed when the data bit is shorter than the transceiver loop delay. */
#define TDC_MIN_DATA_BITRATE 2500000UL
#define TDC_MAX_DATA_BRP 2U
#define TDC_OFFSET_MAX 127U

#define WARNING_LIMIT 96U
#define PASSIVE_LIMIT 128U

#define FDCAN_NOTIFICATIONS                                           \
    (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | \
     FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_MESSAGE_LOST | \
     FDCAN_IT_TX_EVT_FIFO_NEW_DATA | FDCAN_IT_TX_ABORT_COMPLETE |     \
     FDCAN_IT_ERROR_WARNING | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF)

_Static_assert((TX_QUEUE_LEN & (TX_QUEUE_LEN - 1U)) == 0U, "TX_QUEUE_LEN must be a power of two");
_Static_assert(TX_SLOTS < NO_SLOT, "slot index must fit the message marker");

typedef struct {
    FDCAN_HandleTypeDef *hfdcan;
    IRQn_Type irq;
    can_bridge_config_t defaults;

    volatile bool started;
    uint32_t mode_flags;
    volatile uint8_t state; /* enum gs_can_state */
    bool overflow;          /* a frame was lost; flag the next one sent to the host */

    frame_queue_t txq;
    queued_frame_t txq_slots[TX_QUEUE_LEN];
    queued_frame_t inflight[TX_SLOTS];
    bool inflight_used[TX_SLOTS];
    uint8_t hw_buffer_slot[HW_TX_BUFFERS];
} channel_t;

static channel_t channels[CAN_BRIDGE_CHANNELS];

static channel_t *channel_of(const FDCAN_HandleTypeDef *hfdcan, uint8_t *index) {
    for (uint8_t ch = 0U; ch < CAN_BRIDGE_CHANNELS; ch++) {
        if (channels[ch].hfdcan == hfdcan) {
            *index = ch;
            return &channels[ch];
        }
    }
    return NULL;
}

/* ------------------------------------------------------------------------ */
/* Frames to the host                                                       */
/* ------------------------------------------------------------------------ */

/* Sets the wire size and appends the timestamp if the host asked for it. */
static void finish_for_host(channel_t *c, queued_frame_t *q, uint32_t now_us) {
    if (c->overflow) {
        q->frame.flags |= GS_CAN_FLAG_OVERFLOW;
        c->overflow = false;
    }

    const uint32_t data_size = ((q->frame.flags & GS_CAN_FLAG_FD) != 0U) ? GS_HOST_FRAME_FD_DATA
                                                                         : GS_HOST_FRAME_CLASSIC_DATA;
    q->size = GS_HOST_FRAME_HEADER_SIZE + data_size;
    if ((c->mode_flags & GS_CAN_MODE_HW_TIMESTAMP) != 0U) {
        memcpy(&q->frame.payload[data_size], &now_us, sizeof now_us);
        q->size += GS_HOST_FRAME_TIMESTAMP_SIZE;
    }
}

static void echo_to_host(channel_t *c, const queued_frame_t *sent) {
    queued_frame_t *q = gs_usb_in_reserve();
    if (q == NULL) {
        c->overflow = true;
        return;
    }
    q->frame = sent->frame;
    q->frame.reserved = 0U;
    finish_for_host(c, q, timestamp_us());
    gs_usb_in_commit();
}

static uint8_t level_bits(uint32_t tec, uint32_t rec, uint32_t limit, uint8_t tx_bit, uint8_t rx_bit) {
    uint8_t bits = (uint8_t)(((tec >= limit) ? tx_bit : 0U) | ((rec >= limit) ? rx_bit : 0U));
    if (bits == 0U) {
        bits = (tec >= rec) ? tx_bit : rx_bit;
    }
    return bits;
}

static void report_state_change(channel_t *c, uint8_t ch, uint8_t old_state, uint8_t new_state, uint32_t tec, uint32_t rec) {
    queued_frame_t *q = gs_usb_in_reserve();
    if (q == NULL) {
        c->overflow = true;
        return;
    }

    memset(&q->frame, 0, GS_HOST_FRAME_HEADER_SIZE + CAN_ERR_DLC);
    uint8_t *data = q->frame.payload;
    uint32_t id = CAN_ERR_FLAG | CAN_ERR_CNT;

    switch (new_state) {
        case GS_CAN_STATE_BUS_OFF:
            id |= CAN_ERR_BUSOFF;
            break;
        case GS_CAN_STATE_ERROR_PASSIVE:
            id |= CAN_ERR_CRTL;
            data[1] = level_bits(tec, rec, PASSIVE_LIMIT, CAN_ERR_CRTL_TX_PASSIVE, CAN_ERR_CRTL_RX_PASSIVE);
            break;
        case GS_CAN_STATE_ERROR_WARNING:
            id |= CAN_ERR_CRTL;
            data[1] = level_bits(tec, rec, WARNING_LIMIT, CAN_ERR_CRTL_TX_WARNING, CAN_ERR_CRTL_RX_WARNING);
            break;
        default:
            if (old_state == GS_CAN_STATE_BUS_OFF) {
                id |= CAN_ERR_RESTARTED;
            } else {
                id |= CAN_ERR_CRTL;
                data[1] = CAN_ERR_CRTL_ACTIVE;
            }
            break;
    }
    data[6] = (uint8_t)tec;
    data[7] = (uint8_t)rec;

    q->frame.echo_id = GS_HOST_FRAME_ECHO_ID_RX;
    q->frame.can_id = id;
    q->frame.can_dlc = CAN_ERR_DLC;
    q->frame.channel = ch;
    finish_for_host(c, q, timestamp_us());
    gs_usb_in_commit();
}

static void drain_rx(channel_t *c, uint8_t ch, uint32_t fifo, bool lost) {
    static uint8_t discard[GS_HOST_FRAME_FD_DATA];

    if (lost) {
        c->overflow = true;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(c->hfdcan, fifo) > 0U) {
        FDCAN_RxHeaderTypeDef rx;
        queued_frame_t *q = gs_usb_in_reserve();
        uint8_t *dst = (q != NULL) ? q->frame.payload : discard;

        if (HAL_FDCAN_GetRxMessage(c->hfdcan, fifo, &rx, dst) != HAL_OK) {
            break;
        }
        if (q == NULL) {
            c->overflow = true; /* host is not keeping up */
            continue;
        }

        const uint32_t now = timestamp_us();
        const bool extended = rx.IdType == FDCAN_EXTENDED_ID;
        uint32_t id = rx.Identifier & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        if (extended) {
            id |= CAN_EFF_FLAG;
        }
        if (rx.RxFrameType == FDCAN_REMOTE_FRAME) {
            id |= CAN_RTR_FLAG;
        }

        uint8_t flags = 0U;
        if (rx.FDFormat == FDCAN_FD_CAN) {
            flags |= GS_CAN_FLAG_FD;
            if (rx.BitRateSwitch == FDCAN_BRS_ON) {
                flags |= GS_CAN_FLAG_BRS;
            }
            if (rx.ErrorStateIndicator == FDCAN_ESI_PASSIVE) {
                flags |= GS_CAN_FLAG_ESI;
            }
        }

        q->frame.echo_id = GS_HOST_FRAME_ECHO_ID_RX;
        q->frame.can_id = id;
        q->frame.can_dlc = (uint8_t)rx.DataLength;
        q->frame.channel = ch;
        q->frame.flags = flags;
        q->frame.reserved = 0U;
        finish_for_host(c, q, now);
        gs_usb_in_commit();
        leds_activity(ch, LED_RX);
    }
}

/* ------------------------------------------------------------------------ */
/* Frames to the bus                                                        */
/* ------------------------------------------------------------------------ */

static bool build_tx_header(const channel_t *c, const struct gs_host_frame *f, FDCAN_TxHeaderTypeDef *tx) {
    const bool fd = (f->flags & GS_CAN_FLAG_FD) != 0U;

    if ((f->can_dlc > 15U) || ((f->can_id & CAN_ERR_FLAG) != 0U)) {
        return false;
    }
    if (fd && ((c->mode_flags & GS_CAN_MODE_FD) == 0U)) {
        return false;
    }

    if ((f->can_id & CAN_EFF_FLAG) != 0U) {
        tx->IdType = FDCAN_EXTENDED_ID;
        tx->Identifier = f->can_id & CAN_EFF_MASK;
    } else {
        tx->IdType = FDCAN_STANDARD_ID;
        tx->Identifier = f->can_id & CAN_SFF_MASK;
    }
    tx->TxFrameType = (!fd && ((f->can_id & CAN_RTR_FLAG) != 0U)) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    tx->DataLength = f->can_dlc;
    tx->ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx->BitRateSwitch = (fd && ((f->flags & GS_CAN_FLAG_BRS) != 0U)) ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx->FDFormat = fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    tx->TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    tx->MessageMarker = 0U;
    return true;
}

static uint8_t find_free_slot(const channel_t *c) {
    for (uint8_t slot = 0U; slot < TX_SLOTS; slot++) {
        if (!c->inflight_used[slot]) {
            return slot;
        }
    }
    return NO_SLOT;
}

static void service_tx(channel_t *c, uint8_t ch) {
    bool consumed = false;

    while (c->started && (HAL_FDCAN_GetTxFifoFreeLevel(c->hfdcan) > 0U)) {
        queued_frame_t *q = frame_queue_peek(&c->txq);
        if (q == NULL) {
            break;
        }
        const uint8_t slot = find_free_slot(c);
        if (slot == NO_SLOT) {
            break;
        }

        FDCAN_TxHeaderTypeDef tx;
        if (!build_tx_header(c, &q->frame, &tx)) {
            /* Not sendable in the current mode. Echo it anyway so the host
             * does not wait forever for this TX context. */
            echo_to_host(c, q);
            frame_queue_pop(&c->txq);
            consumed = true;
            continue;
        }

        c->inflight[slot] = *q;
        tx.MessageMarker = slot;
        if (HAL_FDCAN_AddMessageToTxFifoQ(c->hfdcan, &tx, q->frame.payload) != HAL_OK) {
            break;
        }
        const uint32_t buffer = (uint32_t)__builtin_ctz(HAL_FDCAN_GetLatestTxFifoQRequestBuffer(c->hfdcan));
        if (buffer < HW_TX_BUFFERS) {
            c->hw_buffer_slot[buffer] = slot;
        }
        c->inflight_used[slot] = true;

        frame_queue_pop(&c->txq);
        consumed = true;
        leds_activity(ch, LED_TX);
    }

    if (consumed) {
        gs_usb_notify_tx_space();
    }
}

static void complete_tx(channel_t *c, uint8_t slot) {
    if ((slot >= TX_SLOTS) || !c->inflight_used[slot]) {
        return;
    }
    c->inflight_used[slot] = false;
    echo_to_host(c, &c->inflight[slot]);
}

static void release_inflight(channel_t *c) {
    for (uint8_t slot = 0U; slot < TX_SLOTS; slot++) {
        c->inflight_used[slot] = false;
    }
    for (uint8_t b = 0U; b < HW_TX_BUFFERS; b++) {
        c->hw_buffer_slot[b] = NO_SLOT;
    }
}

/* ------------------------------------------------------------------------ */
/* HAL callbacks (FDCAN interrupt context)                                  */
/* ------------------------------------------------------------------------ */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    uint8_t ch;
    channel_t *c = channel_of(hfdcan, &ch);
    if (c != NULL) {
        drain_rx(c, ch, FDCAN_RX_FIFO0, (RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U);
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    uint8_t ch;
    channel_t *c = channel_of(hfdcan, &ch);
    if (c != NULL) {
        drain_rx(c, ch, FDCAN_RX_FIFO1, (RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0U);
    }
}

void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs) {
    (void)TxEventFifoITs;
    uint8_t ch;
    channel_t *c = channel_of(hfdcan, &ch);
    if (c == NULL) {
        return;
    }

    while ((hfdcan->Instance->TXEFS & FDCAN_TXEFS_EFFL) != 0U) {
        FDCAN_TxEventFifoTypeDef event;
        if (HAL_FDCAN_GetTxEvent(hfdcan, &event) != HAL_OK) {
            break;
        }
        complete_tx(c, (uint8_t)event.MessageMarker);
    }
    service_tx(c, ch);
}

/* One-shot mode: a failed transmission is cancelled instead of retried. */
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes) {
    uint8_t ch;
    channel_t *c = channel_of(hfdcan, &ch);
    if (c == NULL) {
        return;
    }

    for (uint8_t b = 0U; b < HW_TX_BUFFERS; b++) {
        if ((BufferIndexes & (1UL << b)) != 0U) {
            complete_tx(c, c->hw_buffer_slot[b]);
            c->hw_buffer_slot[b] = NO_SLOT;
        }
    }
    service_tx(c, ch);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    (void)ErrorStatusITs;
    uint8_t ch;
    channel_t *c = channel_of(hfdcan, &ch);
    if (c == NULL) {
        return;
    }

    const uint32_t psr = hfdcan->Instance->PSR;
    const uint32_t ecr = hfdcan->Instance->ECR;
    const uint32_t tec = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
    const uint32_t rec = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;

    uint8_t state = GS_CAN_STATE_ERROR_ACTIVE;
    if ((psr & FDCAN_PSR_BO) != 0U) {
        state = GS_CAN_STATE_BUS_OFF;
    } else if ((psr & FDCAN_PSR_EP) != 0U) {
        state = GS_CAN_STATE_ERROR_PASSIVE;
    } else if ((psr & FDCAN_PSR_EW) != 0U) {
        state = GS_CAN_STATE_ERROR_WARNING;
    }

    if (state != c->state) {
        report_state_change(c, ch, c->state, state, tec, rec);
        c->state = state;
        leds_set_fault(ch, state == GS_CAN_STATE_BUS_OFF);
    }

    /* The controller enters INIT on bus-off. Leaving INIT starts the
     * standard recovery (128 x 11 recessive bits). */
    if ((state == GS_CAN_STATE_BUS_OFF) && c->started) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

static void timing_from_init(struct gs_device_bittiming *bt, uint32_t seg1, uint32_t seg2, uint32_t sjw, uint32_t brp) {
    bt->prop_seg = 0U;
    bt->phase_seg1 = seg1;
    bt->phase_seg2 = seg2;
    bt->sjw = sjw;
    bt->brp = brp;
}

void can_bridge_init(void) {
    FDCAN_HandleTypeDef *const handles[CAN_BRIDGE_CHANNELS] = { &hfdcan1, &hfdcan2, &hfdcan3 };
    const IRQn_Type irqs[CAN_BRIDGE_CHANNELS] = { FDCAN1_IT0_IRQn, FDCAN2_IT0_IRQn, FDCAN3_IT0_IRQn };

    for (uint8_t ch = 0U; ch < CAN_BRIDGE_CHANNELS; ch++) {
        channel_t *c = &channels[ch];
        c->hfdcan = handles[ch];
        c->irq = irqs[ch];
        c->started = false;
        c->state = GS_CAN_STATE_STOPPED;
        frame_queue_init(&c->txq, c->txq_slots, TX_QUEUE_LEN);
        release_inflight(c);

        const FDCAN_InitTypeDef *init = &c->hfdcan->Init;
        timing_from_init(&c->defaults.nominal, init->NominalTimeSeg1, init->NominalTimeSeg2, init->NominalSyncJumpWidth, init->NominalPrescaler);
        timing_from_init(&c->defaults.data, init->DataTimeSeg1, init->DataTimeSeg2, init->DataSyncJumpWidth, init->DataPrescaler);
        c->defaults.flags = (init->FrameFormat != FDCAN_FRAME_CLASSIC) ? GS_CAN_MODE_FD : GS_CAN_MODE_NORMAL;
    }
}

void can_bridge_default_config(uint8_t channel, can_bridge_config_t *config) {
    if (channel < CAN_BRIDGE_CHANNELS) {
        *config = channels[channel].defaults;
    }
}

static bool in_range(uint32_t value, uint32_t min, uint32_t max) {
    return (value >= min) && (value <= max);
}

bool can_bridge_nominal_timing_valid(const struct gs_device_bittiming *bt) {
    return (bt->prop_seg <= CAN_BRIDGE_NTSEG1_MAX) && (bt->phase_seg1 <= CAN_BRIDGE_NTSEG1_MAX) &&
           in_range(bt->prop_seg + bt->phase_seg1, CAN_BRIDGE_NTSEG1_MIN, CAN_BRIDGE_NTSEG1_MAX) &&
           in_range(bt->phase_seg2, CAN_BRIDGE_NTSEG2_MIN, CAN_BRIDGE_NTSEG2_MAX) &&
           in_range(bt->sjw, 1U, CAN_BRIDGE_NSJW_MAX) && in_range(bt->brp, 1U, CAN_BRIDGE_NBRP_MAX);
}

bool can_bridge_data_timing_valid(const struct gs_device_bittiming *bt) {
    return (bt->prop_seg <= CAN_BRIDGE_DTSEG1_MAX) && (bt->phase_seg1 <= CAN_BRIDGE_DTSEG1_MAX) &&
           in_range(bt->prop_seg + bt->phase_seg1, CAN_BRIDGE_DTSEG1_MIN, CAN_BRIDGE_DTSEG1_MAX) &&
           in_range(bt->phase_seg2, CAN_BRIDGE_DTSEG2_MIN, CAN_BRIDGE_DTSEG2_MAX) &&
           in_range(bt->sjw, 1U, CAN_BRIDGE_DSJW_MAX) && in_range(bt->brp, 1U, CAN_BRIDGE_DBRP_MAX);
}

static uint32_t fdcan_mode(uint32_t flags) {
    const bool listen = (flags & GS_CAN_MODE_LISTEN_ONLY) != 0U;
    const bool loop = (flags & GS_CAN_MODE_LOOP_BACK) != 0U;
    if (listen && loop) {
        return FDCAN_MODE_INTERNAL_LOOPBACK;
    }
    if (listen) {
        return FDCAN_MODE_BUS_MONITORING;
    }
    if (loop) {
        return FDCAN_MODE_EXTERNAL_LOOPBACK;
    }
    return FDCAN_MODE_NORMAL;
}

static bool configure_tdc(FDCAN_HandleTypeDef *hfdcan, const can_bridge_config_t *config) {
    if ((config->flags & GS_CAN_MODE_FD) == 0U) {
        return true;
    }

    const struct gs_device_bittiming *d = &config->data;
    const uint32_t tseg1 = d->prop_seg + d->phase_seg1;
    const uint32_t bitrate = CAN_BRIDGE_FDCAN_CLOCK_HZ / (d->brp * (1U + tseg1 + d->phase_seg2));
    if ((bitrate <= TDC_MIN_DATA_BITRATE) || (d->brp > TDC_MAX_DATA_BRP)) {
        return true;
    }

    /* Secondary sample point at the data sample point, in FDCAN clock periods. */
    uint32_t offset = d->brp * (1U + tseg1);
    if (offset > TDC_OFFSET_MAX) {
        offset = TDC_OFFSET_MAX;
    }
    return (HAL_FDCAN_ConfigTxDelayCompensation(hfdcan, offset, 0U) == HAL_OK) &&
           (HAL_FDCAN_EnableTxDelayCompensation(hfdcan) == HAL_OK);
}

bool can_bridge_start(uint8_t channel, const can_bridge_config_t *config) {
    if (channel >= CAN_BRIDGE_CHANNELS) {
        return false;
    }
    const bool fd = (config->flags & GS_CAN_MODE_FD) != 0U;
    if (!can_bridge_nominal_timing_valid(&config->nominal) ||
        (fd && !can_bridge_data_timing_valid(&config->data))) {
        return false;
    }

    channel_t *c = &channels[channel];
    FDCAN_HandleTypeDef *h = c->hfdcan;

    HAL_NVIC_DisableIRQ(c->irq);
    if (c->started) {
        c->started = false;
        (void)HAL_FDCAN_Stop(h);
    }
    release_inflight(c);

    h->Init.FrameFormat = fd ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;
    h->Init.Mode = fdcan_mode(config->flags);
    h->Init.AutoRetransmission = ((config->flags & GS_CAN_MODE_ONE_SHOT) != 0U) ? DISABLE : ENABLE;
    h->Init.NominalPrescaler = config->nominal.brp;
    h->Init.NominalTimeSeg1 = config->nominal.prop_seg + config->nominal.phase_seg1;
    h->Init.NominalTimeSeg2 = config->nominal.phase_seg2;
    h->Init.NominalSyncJumpWidth = config->nominal.sjw;
    if (fd) {
        h->Init.DataPrescaler = config->data.brp;
        h->Init.DataTimeSeg1 = config->data.prop_seg + config->data.phase_seg1;
        h->Init.DataTimeSeg2 = config->data.phase_seg2;
        h->Init.DataSyncJumpWidth = config->data.sjw;
    }
    h->Init.StdFiltersNbr = 0U;
    h->Init.ExtFiltersNbr = 0U;
    h->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    c->mode_flags = config->flags;
    c->overflow = false;

    const bool ok =
        (HAL_FDCAN_Init(h) == HAL_OK) &&
        (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO1, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) == HAL_OK) &&
        configure_tdc(h, config) &&
        (HAL_FDCAN_ActivateNotification(h, FDCAN_NOTIFICATIONS, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) == HAL_OK) &&
        (HAL_FDCAN_Start(h) == HAL_OK);

    c->state = ok ? GS_CAN_STATE_ERROR_ACTIVE : GS_CAN_STATE_STOPPED;
    c->started = ok;
    leds_set_fault(channel, false);
    HAL_NVIC_EnableIRQ(c->irq);

    if (ok) {
        can_bridge_kick_tx(channel); /* frames the host queued before the start */
    }
    return ok;
}

void can_bridge_stop(uint8_t channel) {
    if (channel >= CAN_BRIDGE_CHANNELS) {
        return;
    }
    channel_t *c = &channels[channel];

    HAL_NVIC_DisableIRQ(c->irq);
    if (c->started) {
        c->started = false;
        (void)HAL_FDCAN_Stop(c->hfdcan);
    }
    release_inflight(c);
    frame_queue_flush(&c->txq); /* safe: the queue's consumer interrupt is disabled */
    c->state = GS_CAN_STATE_STOPPED;
    leds_set_fault(channel, false);
    HAL_NVIC_EnableIRQ(c->irq);

    gs_usb_notify_tx_space();
}

bool can_bridge_is_started(uint8_t channel) {
    return (channel < CAN_BRIDGE_CHANNELS) && channels[channel].started;
}

void can_bridge_get_state(uint8_t channel, struct gs_device_state *state) {
    state->state = GS_CAN_STATE_STOPPED;
    state->rxerr = 0U;
    state->txerr = 0U;
    if (!can_bridge_is_started(channel)) {
        return;
    }

    const channel_t *c = &channels[channel];
    const uint32_t ecr = c->hfdcan->Instance->ECR;
    state->state = c->state;
    state->txerr = (ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos;
    state->rxerr = (ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos;
}

frame_queue_t *can_bridge_tx_queue(uint8_t channel) {
    return (channel < CAN_BRIDGE_CHANNELS) ? &channels[channel].txq : NULL;
}

void can_bridge_kick_tx(uint8_t channel) {
    if (channel < CAN_BRIDGE_CHANNELS) {
        NVIC_SetPendingIRQ(channels[channel].irq);
    }
}

void can_bridge_irq(uint8_t channel) {
    if (channel < CAN_BRIDGE_CHANNELS) {
        service_tx(&channels[channel], channel);
    }
}
