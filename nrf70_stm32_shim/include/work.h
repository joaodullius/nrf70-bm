/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Cooperative work-queue abstraction for STM32 bare-metal.
 *        Mirrors nrf70_zephyr_shim/include/work.h interface.
 *        No threads — items are executed by nrf70_process_work().
 *
 * USAGE: Call nrf70_process_work() from the main loop and from the
 *        HOST_IRQ EXTI handler to drain the queue cooperatively.
 */

#ifndef __STM32_WORK_H__
#define __STM32_WORK_H__

#include <stdint.h>
#include <stdbool.h>
#include "autoconf.h"
#include "osal_structs.h"

struct stm32_work_item {
    bool     in_use;
    void   (*callback)(unsigned long data);
    unsigned long data;
    enum nrf_wifi_tasklet_type type;
    bool     pending;
};

/**
 * Allocate a work item from the static pool.
 * @param type  Tasklet type (BH, IRQ, TX_DONE, RX).
 * @return Pointer to work item, or NULL if pool exhausted.
 */
struct stm32_work_item *work_alloc(int type);

/** Initialise callback and data fields of a previously allocated item. */
void work_init(struct stm32_work_item *item,
               void (*callback)(unsigned long data),
               unsigned long data);

/** Enqueue a work item for execution. */
void work_schedule(struct stm32_work_item *item);

/** Cancel a pending work item (marks it not-pending; does not free it). */
void work_kill(struct stm32_work_item *item);

/** Return a work item to the pool. */
void work_free(struct stm32_work_item *item);

/**
 * @brief Execute all pending work items in the queue.
 *        Must be called from the main loop.
 *        Also call from the HOST_IRQ EXTI handler to process IRQ work inline.
 */
void nrf70_process_work(void);

#endif /* __STM32_WORK_H__ */
