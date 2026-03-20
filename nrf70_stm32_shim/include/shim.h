/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief STM32 bare-metal shim top-level declarations.
 *        Mirrors nrf70_zephyr_shim/include/shim.h.
 */

#ifndef __STM32_SHIM_H__
#define __STM32_SHIM_H__

#include <stdint.h>
#include <stdbool.h>
#include "osal_ops.h"

/* Exported OSAL ops struct — pass to nrf_wifi_osal_init() */
extern const struct nrf_wifi_osal_ops nrf_wifi_os_bm_ops;

/* Private bus context (analogous to zep_shim_bus_qspi_priv) */
struct stm32_shim_bus_priv {
    bool dev_added;
    bool dev_init;
};

/* Linked-list node (replaces Zephyr sys_dlist_t) */
struct stm32_llist_node {
    struct stm32_llist_node *next;
    void *data;
};

/* Linked list head */
struct stm32_llist {
    struct stm32_llist_node *head;
    unsigned int len;
};

/* Interrupt callback private context */
struct stm32_intr_priv {
    void *callbk_data;
    int (*callbk_fn)(void *callbk_data);
};

#endif /* __STM32_SHIM_H__ */
