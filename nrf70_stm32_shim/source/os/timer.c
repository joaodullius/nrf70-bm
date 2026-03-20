/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Software timer implementation backed by HAL_GetTick().
 *        Mirrors nrf70_zephyr_shim/source/os/timer.c interface.
 *
 * Timers are serviced cooperatively — call nrf70_process_timers() from the
 * main loop (and optionally from the HOST_IRQ ISR) to fire expired callbacks.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "autoconf.h"
#include "nrf70_bm_board_config.h"
#include "timer.h"

/* Static pool of software timers */
static struct timer_list *timer_pool[CONFIG_NRF70_MAX_TIMERS];
static int timer_pool_count = 0;

void init_timer(struct timer_list *timer)
{
    timer->active    = false;
    timer->expire_ms = 0;

    /* Register in pool so nrf70_process_timers() can find it */
    if (timer_pool_count < CONFIG_NRF70_MAX_TIMERS) {
        timer_pool[timer_pool_count++] = timer;
    } else {
        printf("[nrf70 timer] Timer pool full!\r\n");
    }
}

void mod_timer(struct timer_list *timer, int msec)
{
    timer->expire_ms = HAL_GetTick() + (uint32_t)msec;
    timer->active    = true;
}

void del_timer_sync(struct timer_list *timer)
{
    timer->active = false;
}

/**
 * @brief Fire all expired timers.
 *        Call this periodically from the main superloop.
 */
void nrf70_process_timers(void)
{
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < timer_pool_count; i++) {
        struct timer_list *t = timer_pool[i];

        if (t == NULL || !t->active) {
            continue;
        }

        /* Handle 32-bit wrap-around correctly with unsigned subtraction */
        if ((now - t->expire_ms) < 0x80000000UL) {
            t->active = false;
            if (t->function != NULL) {
                t->function(t->data);
            }
        }
    }
}
