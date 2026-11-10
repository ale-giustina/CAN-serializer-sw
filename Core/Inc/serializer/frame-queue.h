/**
 * @file frame-queue.h
 * @brief Lock-free single-producer / single-consumer queue of gs_usb frames.
 *
 * @details Safe between one producer context and one consumer context (two
 * interrupt handlers, or an interrupt handler and the main loop). The
 * producer calls reserve() then commit(); the consumer calls peek() then pop().
 * The capacity must be a power of two.
 */

#ifndef FRAME_QUEUE_H
#define FRAME_QUEUE_H

#include <stddef.h>
#include <stdint.h>

#include "gs-usb-protocol.h"
#include "main.h"

typedef struct {
    struct gs_host_frame frame;
    uint32_t size; /* bytes of `frame` that are valid on the wire */
} queued_frame_t;

typedef struct {
    queued_frame_t *slots;
    uint32_t mask;
    volatile uint32_t head; /* written by the producer only */
    volatile uint32_t tail; /* written by the consumer only */
} frame_queue_t;

static inline void frame_queue_init(frame_queue_t *q, queued_frame_t *slots, uint32_t capacity) {
    q->slots = slots;
    q->mask = capacity - 1U;
    q->head = 0U;
    q->tail = 0U;
}

static inline uint32_t frame_queue_count(const frame_queue_t *q) {
    return q->head - q->tail;
}

static inline uint32_t frame_queue_space(const frame_queue_t *q) {
    return (q->mask + 1U) - frame_queue_count(q);
}

/** Producer: slot to fill, or NULL when the queue is full. */
static inline queued_frame_t *frame_queue_reserve(frame_queue_t *q) {
    if (frame_queue_count(q) > q->mask) {
        return NULL;
    }
    return &q->slots[q->head & q->mask];
}

/** Producer: publish the slot returned by reserve(). */
static inline void frame_queue_commit(frame_queue_t *q) {
    __DMB();
    q->head = q->head + 1U;
}

/** Consumer: oldest frame, or NULL when the queue is empty. */
static inline queued_frame_t *frame_queue_peek(frame_queue_t *q) {
    if (q->head == q->tail) {
        return NULL;
    }
    __DMB();
    return &q->slots[q->tail & q->mask];
}

/** Consumer: release the frame returned by peek(). */
static inline void frame_queue_pop(frame_queue_t *q) {
    __DMB();
    q->tail = q->tail + 1U;
}

/** Consumer: drop everything currently queued. */
static inline void frame_queue_flush(frame_queue_t *q) {
    q->tail = q->head;
}

#endif /* FRAME_QUEUE_H */
