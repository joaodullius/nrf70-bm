/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Software timer abstraction backed by HAL_GetTick().
 *        Mirrors nrf70_zephyr_shim/include/timer.h interface.
 *
 * USAGE: Call nrf70_process_timers() from the main loop (and optionally from
 *        the EXTI ISR) to fire expired timers cooperatively.
 */

#ifndef __STM32_TIMER_H__
#define __STM32_TIMER_H__

#include <stdint.h>
#include <stdbool.h>
#include "autoconf.h"

struct timer_list {
    void (*function)(unsigned long data);
    unsigned long data;
    bool          active;
    uint32_t      expire_ms;   /* absolute HAL_GetTick() target */
};

/** Initialise a timer_list (sets active=false). */
void init_timer(struct timer_list *timer);

/** Arm the timer: fires after @p msec milliseconds from now. */
void mod_timer(struct timer_list *timer, int msec);

/** Cancel a pending timer. */
void del_timer_sync(struct timer_list *timer);

/**
 * @brief Service all software timers.
 *        Must be called periodically from the main loop.
 *        Also safe to call from the HOST_IRQ EXTI handler.
 */
void nrf70_process_timers(void);

#endif /* __STM32_TIMER_H__ */
