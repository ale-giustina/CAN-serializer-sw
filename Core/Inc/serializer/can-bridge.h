/**
 * @file can-bridge.h
 * @brief Runs the three FDCAN channels on behalf of the gs_usb host.
 *
 * @details
 * Execution contexts:
 * - can_bridge_start() / can_bridge_stop(): main loop only (they call
 *   HAL_FDCAN_Init(), which polls with timeouts).
 * - can_bridge_irq(): from FDCANx_IT0_IRQHandler, before HAL_FDCAN_IRQHandler().
 *   It moves frames from the channel's TX queue into the hardware TX FIFO.
 * - can_bridge_tx_queue() is produced by the USB interrupt; consumed here.
 *
 * Received frames, TX echoes and error frames are pushed to the gs_usb IN
 * queue. All three FDCAN interrupts share one priority, so they never
 * preempt each other and act as a single producer for that queue.
 */

#ifndef CAN_BRIDGE_H
#define CAN_BRIDGE_H

#include <stdbool.h>
#include <stdint.h>

#include "frame-queue.h"
#include "gs-usb-protocol.h"

#define CAN_BRIDGE_CHANNELS 3U
#define CAN_BRIDGE_FDCAN_CLOCK_HZ 80000000U

/* Bit timing limits of the G4 FDCAN, as reported to the host. */
#define CAN_BRIDGE_NTSEG1_MIN 2U
#define CAN_BRIDGE_NTSEG1_MAX 256U
#define CAN_BRIDGE_NTSEG2_MIN 1U
#define CAN_BRIDGE_NTSEG2_MAX 128U
#define CAN_BRIDGE_NSJW_MAX 128U
#define CAN_BRIDGE_NBRP_MAX 512U
#define CAN_BRIDGE_DTSEG1_MIN 1U
#define CAN_BRIDGE_DTSEG1_MAX 32U
#define CAN_BRIDGE_DTSEG2_MIN 1U
#define CAN_BRIDGE_DTSEG2_MAX 16U
#define CAN_BRIDGE_DSJW_MAX 16U
#define CAN_BRIDGE_DBRP_MAX 32U

/* Mode flags the bridge implements. */
#define CAN_BRIDGE_SUPPORTED_MODES                                            \
    (GS_CAN_MODE_LISTEN_ONLY | GS_CAN_MODE_LOOP_BACK | GS_CAN_MODE_ONE_SHOT | \
     GS_CAN_MODE_HW_TIMESTAMP | GS_CAN_MODE_FD)

typedef struct {
    struct gs_device_bittiming nominal;
    struct gs_device_bittiming data;
    uint32_t flags; /* GS_CAN_MODE_* */
} can_bridge_config_t;

/** Reads the CubeMX bit timing as the power-up default and prepares the queues. */
void can_bridge_init(void);

/** Fills `config` with the CubeMX bit timing of `channel` (500 kbit/s / 5 Mbit/s). */
void can_bridge_default_config(uint8_t channel, can_bridge_config_t *config);

bool can_bridge_nominal_timing_valid(const struct gs_device_bittiming *bt);
bool can_bridge_data_timing_valid(const struct gs_device_bittiming *bt);

/** (Re)configures and starts a channel. Main loop only. */
bool can_bridge_start(uint8_t channel, const can_bridge_config_t *config);

/** Stops a channel and drops whatever it still had to send. Main loop only. */
void can_bridge_stop(uint8_t channel);

bool can_bridge_is_started(uint8_t channel);

/** Bus state and error counters, for GS_USB_BREQ_GET_STATE. Any context. */
void can_bridge_get_state(uint8_t channel, struct gs_device_state *state);

/** Host-to-bus queue of `channel`. The USB interrupt is its only producer. */
frame_queue_t *can_bridge_tx_queue(uint8_t channel);

/** Asks the channel's interrupt to load queued frames into the hardware. */
void can_bridge_kick_tx(uint8_t channel);

/** Called from FDCANx_IT0_IRQHandler. */
void can_bridge_irq(uint8_t channel);

#endif /* CAN_BRIDGE_H */
