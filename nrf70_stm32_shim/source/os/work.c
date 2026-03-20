/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Cooperative FIFO work queue for STM32 bare-metal nRF70 shim.
 *        Mirrors nrf70_zephyr_shim/source/os/work.c interface but uses
 *        no RTOS threads — all work is executed by the caller of
 *        nrf70_process_work().
 *
 * Call nrf70_process_work() from:
 *   1. The main superloop (always)
 *   2. The HOST_IRQ EXTI ISR (to run IRQ-type work inline)
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "autoconf.h"
#include "work.h"

/* -------------------------------------------------------------------------
 * Static pool and FIFO queue
 * -------------------------------------------------------------------------*/
static struct stm32_work_item work_pool[CONFIG_NRF70_WORKQ_MAX_ITEMS];

/* Simple circular FIFO of pointers */
static struct stm32_work_item *work_queue[CONFIG_NRF70_WORKQ_MAX_ITEMS];
static volatile int queue_head = 0;  /* index of next item to consume */
static volatile int queue_tail = 0;  /* index of next free slot */

static inline int queue_next(int idx)
{
    return (idx + 1) % CONFIG_NRF70_WORKQ_MAX_ITEMS;
}

static inline bool queue_empty(void)
{
    return queue_head == queue_tail;
}

static inline bool queue_full(void)
{
    return queue_next(queue_tail) == queue_head;
}

/* -------------------------------------------------------------------------
 * Internal: find a free pool slot
 * -------------------------------------------------------------------------*/
static int get_free_work_item_index(void)
{
    for (int i = 0; i < CONFIG_NRF70_WORKQ_MAX_ITEMS; i++) {
        if (!work_pool[i].in_use) {
            return i;
        }
    }
    return -1;
}

/* -------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

struct stm32_work_item *work_alloc(int type)
{
    int idx = get_free_work_item_index();

    if (idx < 0) {
        printf("[nrf70 work] work_alloc: pool exhausted\r\n");
        return NULL;
    }

    work_pool[idx].in_use  = true;
    work_pool[idx].type    = (enum nrf_wifi_tasklet_type)type;
    work_pool[idx].pending = false;

    return &work_pool[idx];
}

void work_init(struct stm32_work_item *item,
               void (*callback)(unsigned long data),
               unsigned long data)
{
    item->callback = callback;
    item->data     = data;
    item->pending  = false;
}

void work_schedule(struct stm32_work_item *item)
{
    if (item == NULL || !item->in_use) {
        return;
    }

    if (item->pending) {
        /* Already queued — do not double-enqueue */
        return;
    }

    if (queue_full()) {
        printf("[nrf70 work] work_schedule: queue full\r\n");
        return;
    }

    item->pending = true;
    work_queue[queue_tail] = item;
    queue_tail = queue_next(queue_tail);
}

void work_kill(struct stm32_work_item *item)
{
    if (item != NULL) {
        item->pending = false;
    }
}

void work_free(struct stm32_work_item *item)
{
    if (item != NULL) {
        item->in_use  = false;
        item->pending = false;
    }
}

/**
 * @brief Drain all pending work items.
 *        Must be called from the main loop and from the EXTI ISR.
 */
void nrf70_process_work(void)
{
    while (!queue_empty()) {
        struct stm32_work_item *item = work_queue[queue_head];
        queue_head = queue_next(queue_head);

        if (item == NULL) {
            continue;
        }

        if (item->pending && item->callback != NULL) {
            item->pending = false;
            item->callback(item->data);
        } else {
            item->pending = false;
        }
    }
}
