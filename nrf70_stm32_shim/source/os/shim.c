/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief OSAL ops implementation for STM32 bare-metal nRF70 shim.
 *        Mirrors nrf70_zephyr_shim/source/os/shim.c.
 *
 * All ~70 function pointers in struct nrf_wifi_osal_ops are implemented here
 * using standard C library (malloc/free), Cortex-M PRIMASK for spinlocks,
 * HAL_Delay for sleep, and DWT cycle counter for microsecond timing.
 *
 * The QSPI-named bus functions are mapped to the SPI implementation because
 * the OSAL uses QSPI naming for the generic SPI-like bus interface.
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "autoconf.h"
#include "nrf70_bm_board_config.h"
#include "shim.h"
#include "work.h"
#include "timer.h"
#include "rpu_hw_if.h"
#include "spi_if.h"
#include "osal_ops.h"

/* -------------------------------------------------------------------------
 * Memory
 * -------------------------------------------------------------------------*/

static void *stm32_mem_alloc(size_t size)
{
    /* Round up to 4-byte alignment (mirrors Zephyr shim) */
    size = (size + 4) & 0xFFFFFFFCU;
    return malloc(size);
}

static void *stm32_mem_zalloc(size_t size)
{
    size = (size + 4) & 0xFFFFFFFCU;
    return calloc(size, 1);
}

static void *stm32_mem_cpy(void *dest, const void *src, size_t count)
{
    return memcpy(dest, src, count);
}

static void *stm32_mem_set(void *start, int val, size_t size)
{
    return memset(start, val, size);
}

static int stm32_mem_cmp(const void *addr1, const void *addr2, size_t size)
{
    return memcmp(addr1, addr2, size);
}

/* -------------------------------------------------------------------------
 * SPI / "QSPI" bus read/write (OSAL uses qspi_ naming for generic SPI bus)
 * -------------------------------------------------------------------------*/

static unsigned int stm32_qspi_read_reg32(void *priv, unsigned long addr)
{
    (void)priv;
    unsigned int val = 0;

    if (addr < 0x0C0000) {
        spim_hl_read((unsigned int)addr, &val, 4);
    } else {
        spim_read((unsigned int)addr, &val, 4);
    }

    return val;
}

static void stm32_qspi_write_reg32(void *priv, unsigned long addr, unsigned int val)
{
    (void)priv;
    spim_write((unsigned int)addr, &val, 4);
}

static void stm32_qspi_cpy_from(void *priv, void *dest, unsigned long addr, size_t count)
{
    (void)priv;
    if (count % 4 != 0) {
        count = (count + 4) & 0xFFFFFFFCU;
    }

    if (addr < 0x0C0000) {
        spim_hl_read((unsigned int)addr, dest, (int)count);
    } else {
        spim_read((unsigned int)addr, dest, (int)count);
    }
}

static void stm32_qspi_cpy_to(void *priv, unsigned long addr, const void *src, size_t count)
{
    (void)priv;
    if (count % 4 != 0) {
        count = (count + 4) & 0xFFFFFFFCU;
    }
    spim_write((unsigned int)addr, src, (int)count);
}

/* SPI-named variants (same implementation) */
static unsigned int stm32_spi_read_reg32(void *priv, unsigned long addr)
{
    return stm32_qspi_read_reg32(priv, addr);
}

static void stm32_spi_write_reg32(void *priv, unsigned long addr, unsigned int val)
{
    stm32_qspi_write_reg32(priv, addr, val);
}

static void stm32_spi_cpy_from(void *priv, void *dest, unsigned long addr, size_t count)
{
    stm32_qspi_cpy_from(priv, dest, addr, count);
}

static void stm32_spi_cpy_to(void *priv, unsigned long addr, const void *src, size_t count)
{
    stm32_qspi_cpy_to(priv, addr, src, count);
}

/* -------------------------------------------------------------------------
 * Spinlocks  — implemented with PRIMASK (Cortex-M global IRQ disable)
 *
 * The driver uses both "normal" and "irq" spinlock variants.
 * In a bare-metal superloop context both simply disable/enable IRQs.
 * -------------------------------------------------------------------------*/

static void *stm32_spinlock_alloc(void)
{
    /* Use a single uint32_t to store the saved PRIMASK value */
    return malloc(sizeof(uint32_t));
}

static void stm32_spinlock_free(void *lock)
{
    free(lock);
}

static void stm32_spinlock_init(void *lock)
{
    if (lock) {
        *(uint32_t *)lock = 0;
    }
}

static void stm32_spinlock_take(void *lock)
{
    (void)lock;
    __disable_irq();
}

static void stm32_spinlock_rel(void *lock)
{
    (void)lock;
    __enable_irq();
}

static void stm32_spinlock_irq_take(void *lock, unsigned long *flags)
{
    (void)lock;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (flags) {
        *flags = (unsigned long)primask;
    }
}

static void stm32_spinlock_irq_rel(void *lock, unsigned long *flags)
{
    (void)lock;
    if (flags && !(*flags & 1U)) {
        __enable_irq();
    }
}

/* -------------------------------------------------------------------------
 * Logging
 * -------------------------------------------------------------------------*/

static int stm32_log_dbg(const char *fmt, va_list args)
{
    char buf[128];
    vsnprintf(buf, sizeof(buf), fmt, args);
    printf("[nrf70 dbg] %s\r\n", buf);
    return 0;
}

static int stm32_log_info(const char *fmt, va_list args)
{
    char buf[128];
    vsnprintf(buf, sizeof(buf), fmt, args);
    printf("[nrf70 inf] %s\r\n", buf);
    return 0;
}

static int stm32_log_err(const char *fmt, va_list args)
{
    char buf[256];
    vsnprintf(buf, sizeof(buf), fmt, args);
    printf("[nrf70 ERR] %s\r\n", buf);
    return 0;
}

/* -------------------------------------------------------------------------
 * Linked list (simple singly-linked, malloc-backed)
 * -------------------------------------------------------------------------*/

static void *stm32_llist_node_alloc(void)
{
    struct stm32_llist_node *node = calloc(1, sizeof(*node));
    return node;
}

static void stm32_llist_node_free(void *node)
{
    free(node);
}

static void *stm32_llist_node_data_get(void *node)
{
    return ((struct stm32_llist_node *)node)->data;
}

static void stm32_llist_node_data_set(void *node, void *data)
{
    ((struct stm32_llist_node *)node)->data = data;
}

static void *stm32_llist_alloc(void)
{
    return calloc(1, sizeof(struct stm32_llist));
}

static void stm32_llist_free(void *llist)
{
    free(llist);
}

static void stm32_llist_init(void *llist)
{
    struct stm32_llist *l = llist;
    l->head = NULL;
    l->len  = 0;
}

static void stm32_llist_add_node_tail(void *llist, void *llist_node)
{
    struct stm32_llist *l      = llist;
    struct stm32_llist_node *n = llist_node;

    n->next = NULL;

    if (l->head == NULL) {
        l->head = n;
    } else {
        struct stm32_llist_node *cur = l->head;
        while (cur->next != NULL) {
            cur = cur->next;
        }
        cur->next = n;
    }
    l->len++;
}

static void stm32_llist_add_node_head(void *llist, void *llist_node)
{
    struct stm32_llist *l      = llist;
    struct stm32_llist_node *n = llist_node;

    n->next = l->head;
    l->head = n;
    l->len++;
}

static void *stm32_llist_get_node_head(void *llist)
{
    return ((struct stm32_llist *)llist)->head;
}

static void *stm32_llist_get_node_nxt(void *llist, void *llist_node)
{
    (void)llist;
    return ((struct stm32_llist_node *)llist_node)->next;
}

static void stm32_llist_del_node(void *llist, void *llist_node)
{
    struct stm32_llist *l      = llist;
    struct stm32_llist_node *n = llist_node;

    if (l->head == n) {
        l->head = n->next;
        l->len--;
        return;
    }

    struct stm32_llist_node *cur = l->head;
    while (cur != NULL && cur->next != n) {
        cur = cur->next;
    }
    if (cur != NULL) {
        cur->next = n->next;
        l->len--;
    }
}

static unsigned int stm32_llist_len(void *llist)
{
    return ((struct stm32_llist *)llist)->len;
}

/* -------------------------------------------------------------------------
 * Network buffers (nwb — mirrors Zephyr shim exactly)
 * -------------------------------------------------------------------------*/
#ifdef CONFIG_NRF70_BM_SCAN_ONLY

struct nwb {
    unsigned char *data;
    unsigned char *tail;
    int len;
    int headroom;
    void *next;
    void *priv;
    int iftype;
    void *ifaddr;
    void *dev;
    int hostbuffer;
    void *cleanup_ctx;
    void (*cleanup_cb)(void);
    unsigned char priority;
    bool chksum_done;
};

static void *stm32_nbuf_alloc(unsigned int size)
{
    struct nwb *nwb = calloc(1, sizeof(*nwb));
    if (!nwb) {
        return NULL;
    }

    nwb->priv = calloc(size, 1);
    if (!nwb->priv) {
        free(nwb);
        return NULL;
    }

    nwb->data     = (unsigned char *)nwb->priv;
    nwb->tail     = nwb->data;
    nwb->len      = 0;
    nwb->headroom = 0;
    nwb->next     = NULL;

    return nwb;
}

static void stm32_nbuf_free(void *nbuf)
{
    if (!nbuf) return;
    free(((struct nwb *)nbuf)->priv);
    free(nbuf);
}

static void stm32_nbuf_headroom_res(void *nbuf, unsigned int size)
{
    struct nwb *nwb = nbuf;
    nwb->data     += size;
    nwb->tail     += size;
    nwb->headroom += size;
}

static unsigned int stm32_nbuf_headroom_get(void *nbuf)
{
    return (unsigned int)((struct nwb *)nbuf)->headroom;
}

static unsigned int stm32_nbuf_data_size(void *nbuf)
{
    return (unsigned int)((struct nwb *)nbuf)->len;
}

static void *stm32_nbuf_data_get(void *nbuf)
{
    return ((struct nwb *)nbuf)->data;
}

static void *stm32_nbuf_data_put(void *nbuf, unsigned int size)
{
    struct nwb *nwb = nbuf;
    unsigned char *ret = nwb->tail;
    nwb->tail += size;
    nwb->len  += size;
    return ret;
}

static void *stm32_nbuf_data_push(void *nbuf, unsigned int size)
{
    struct nwb *nwb = nbuf;
    nwb->data     -= size;
    nwb->headroom -= size;
    nwb->len      += size;
    return nwb->data;
}

static void *stm32_nbuf_data_pull(void *nbuf, unsigned int size)
{
    struct nwb *nwb = nbuf;
    nwb->data     += size;
    nwb->headroom += size;
    nwb->len      -= size;
    return nwb->data;
}

static unsigned char stm32_nbuf_get_priority(void *nbuf)
{
    return ((struct nwb *)nbuf)->priority;
}

static unsigned char stm32_nbuf_get_chksum_done(void *nbuf)
{
    return (unsigned char)((struct nwb *)nbuf)->chksum_done;
}

static void stm32_nbuf_set_chksum_done(void *nbuf, unsigned char chksum_done)
{
    ((struct nwb *)nbuf)->chksum_done = (bool)chksum_done;
}
#endif /* CONFIG_NRF70_BM_SCAN_ONLY */

/* -------------------------------------------------------------------------
 * Tasklets  — delegate to work.c cooperative queue
 * -------------------------------------------------------------------------*/

static void *stm32_tasklet_alloc(int type)
{
    return work_alloc(type);
}

static void stm32_tasklet_free(void *tasklet)
{
    work_free((struct stm32_work_item *)tasklet);
}

static void stm32_tasklet_init(void *tasklet,
                                void (*callback)(unsigned long),
                                unsigned long data)
{
    work_init((struct stm32_work_item *)tasklet, callback, data);
}

static void stm32_tasklet_schedule(void *tasklet)
{
    work_schedule((struct stm32_work_item *)tasklet);
}

static void stm32_tasklet_kill(void *tasklet)
{
    work_kill((struct stm32_work_item *)tasklet);
}

/* -------------------------------------------------------------------------
 * Timing
 * -------------------------------------------------------------------------*/

static int stm32_sleep_ms(int msecs)
{
    HAL_Delay((uint32_t)msecs);
    return 0;
}

static int stm32_delay_us(int usecs)
{
    /* Busy-wait using DWT cycle counter if enabled, else use HAL_Delay(1ms) */
#if defined(DWT) && defined(CoreDebug)
    if (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) {
        uint32_t cycles  = (uint32_t)usecs * (SystemCoreClock / 1000000U);
        uint32_t start   = DWT->CYCCNT;
        while ((DWT->CYCCNT - start) < cycles) {
            /* busy wait */
        }
        return 0;
    }
#endif
    /* Fallback: 1 ms resolution */
    if (usecs >= 1000) {
        HAL_Delay((uint32_t)(usecs / 1000));
    }
    return 0;
}

static unsigned long stm32_time_get_curr_us(void)
{
    /* HAL_GetTick() returns milliseconds; convert to microseconds */
    return (unsigned long)(HAL_GetTick() * 1000UL);
}

static unsigned int stm32_time_elapsed_us(unsigned long start_time_us)
{
    return (unsigned int)(stm32_time_get_curr_us() - start_time_us);
}

static unsigned long stm32_time_get_curr_ms(void)
{
    return (unsigned long)HAL_GetTick();
}

static unsigned int stm32_time_elapsed_ms(unsigned long start_time_ms)
{
    return (unsigned int)(HAL_GetTick() - (uint32_t)start_time_ms);
}

/* -------------------------------------------------------------------------
 * Timer ops  — delegate to timer.c
 * -------------------------------------------------------------------------*/

static void *stm32_timer_alloc(void)
{
    struct timer_list *t = malloc(sizeof(*t));
    if (t) {
        init_timer(t);
    }
    return t;
}

static void stm32_timer_free(void *timer)
{
    free(timer);
}

static void stm32_timer_init(void *timer,
                              void (*callback)(unsigned long),
                              unsigned long data)
{
    struct timer_list *t = timer;
    t->function = callback;
    t->data     = data;
    init_timer(t);
}

static void stm32_timer_schedule(void *timer, unsigned long duration)
{
    mod_timer((struct timer_list *)timer, (int)duration);
}

static void stm32_timer_kill(void *timer)
{
    del_timer_sync((struct timer_list *)timer);
}

/* -------------------------------------------------------------------------
 * Bus: QSPI-named SPI bus ops
 * -------------------------------------------------------------------------*/

static struct stm32_shim_bus_priv g_bus_priv;

/* Interrupt private context — filled by bus_qspi_dev_intr_reg */
static struct stm32_intr_priv g_intr_priv;

static void *stm32_bus_qspi_init(void)
{
    memset(&g_bus_priv, 0, sizeof(g_bus_priv));
    return &g_bus_priv;
}

static void stm32_bus_qspi_deinit(void *os_qspi_priv)
{
    (void)os_qspi_priv;
    memset(&g_bus_priv, 0, sizeof(g_bus_priv));
}

static void *stm32_bus_qspi_dev_add(void *qspi_priv, void *osal_qspi_dev_ctx)
{
    (void)osal_qspi_dev_ctx;
    struct stm32_shim_bus_priv *priv = qspi_priv;
    int ret;

    ret = rpu_init();
    if (ret) {
        printf("[nrf70 shim] rpu_init failed: %d\r\n", ret);
        return NULL;
    }

    ret = rpu_enable();
    if (ret) {
        printf("[nrf70 shim] rpu_enable failed: %d\r\n", ret);
        return NULL;
    }

    priv->dev_added = true;
    return priv;
}

static void stm32_bus_qspi_dev_rem(void *priv)
{
    (void)priv;
    rpu_disable();
}

static enum nrf_wifi_status stm32_bus_qspi_dev_init(void *os_qspi_dev_ctx)
{
    (void)os_qspi_dev_ctx;
    return NRF_WIFI_STATUS_SUCCESS;
}

static void stm32_bus_qspi_dev_deinit(void *priv)
{
    (void)priv;
}

static enum nrf_wifi_status stm32_bus_qspi_intr_reg(void *os_dev_ctx,
                                                     void *callbk_data,
                                                     int (*callbk_fn)(void *callbk_data))
{
    (void)os_dev_ctx;
    int ret;

    g_intr_priv.callbk_data = callbk_data;
    g_intr_priv.callbk_fn   = callbk_fn;

    ret = rpu_irq_config(callbk_data, callbk_fn);
    if (ret) {
        printf("[nrf70 shim] rpu_irq_config failed: %d\r\n", ret);
        return NRF_WIFI_STATUS_FAIL;
    }

    return NRF_WIFI_STATUS_SUCCESS;
}

static void stm32_bus_qspi_intr_unreg(void *os_qspi_dev_ctx)
{
    (void)os_qspi_dev_ctx;
    rpu_irq_remove();
}

static void stm32_bus_qspi_dev_host_map_get(void *os_qspi_dev_ctx,
                                             struct nrf_wifi_osal_host_map *host_map)
{
    (void)os_qspi_dev_ctx;
    if (host_map) {
        host_map->addr = 0;
        host_map->size = 0;
    }
}

/* -------------------------------------------------------------------------
 * Bus: SPI-named ops (same implementation, different pointer names)
 * -------------------------------------------------------------------------*/

static void *stm32_bus_spi_init(void)
{
    return stm32_bus_qspi_init();
}

static void stm32_bus_spi_deinit(void *os_spi_priv)
{
    stm32_bus_qspi_deinit(os_spi_priv);
}

static void *stm32_bus_spi_dev_add(void *spi_priv, void *osal_spi_dev_ctx)
{
    return stm32_bus_qspi_dev_add(spi_priv, osal_spi_dev_ctx);
}

static void stm32_bus_spi_dev_rem(void *os_spi_dev_ctx)
{
    stm32_bus_qspi_dev_rem(os_spi_dev_ctx);
}

static enum nrf_wifi_status stm32_bus_spi_dev_init(void *os_spi_dev_ctx)
{
    return stm32_bus_qspi_dev_init(os_spi_dev_ctx);
}

static void stm32_bus_spi_dev_deinit(void *os_spi_dev_ctx)
{
    stm32_bus_qspi_dev_deinit(os_spi_dev_ctx);
}

static enum nrf_wifi_status stm32_bus_spi_intr_reg(void *os_spi_dev_ctx,
                                                    void *callbk_data,
                                                    int (*callbk_fn)(void *callbk_data))
{
    return stm32_bus_qspi_intr_reg(os_spi_dev_ctx, callbk_data, callbk_fn);
}

static void stm32_bus_spi_intr_unreg(void *os_spi_dev_ctx)
{
    stm32_bus_qspi_intr_unreg(os_spi_dev_ctx);
}

static void stm32_bus_spi_dev_host_map_get(void *os_spi_dev_ctx,
                                            struct nrf_wifi_osal_host_map *host_map)
{
    stm32_bus_qspi_dev_host_map_get(os_spi_dev_ctx, host_map);
}

/* -------------------------------------------------------------------------
 * Power-save bus ops (only when CONFIG_NRF_WIFI_LOW_POWER is defined)
 * -------------------------------------------------------------------------*/
#ifdef CONFIG_NRF_WIFI_LOW_POWER
static int stm32_bus_qspi_ps_sleep(void *os_qspi_priv)
{
    (void)os_qspi_priv;
    return rpu_sleep();
}

static int stm32_bus_qspi_ps_wake(void *os_qspi_priv)
{
    (void)os_qspi_priv;
    return rpu_wakeup();
}

static int stm32_bus_qspi_ps_status(void *os_qspi_priv)
{
    (void)os_qspi_priv;
    return rpu_sleep_status();
}
#endif /* CONFIG_NRF_WIFI_LOW_POWER */

/* -------------------------------------------------------------------------
 * Miscellaneous
 * -------------------------------------------------------------------------*/

static void stm32_assert(int test_val, int val,
                          enum nrf_wifi_assert_op_type op,
                          char *msg)
{
    bool pass = false;

    switch (op) {
    case NRF_WIFI_ASSERT_EQUAL_TO:             pass = (test_val == val); break;
    case NRF_WIFI_ASSERT_NOT_EQUAL_TO:         pass = (test_val != val); break;
    case NRF_WIFI_ASSERT_LESS_THAN:            pass = (test_val <  val); break;
    case NRF_WIFI_ASSERT_LESS_THAN_EQUAL_TO:   pass = (test_val <= val); break;
    case NRF_WIFI_ASSERT_GREATER_THAN:         pass = (test_val >  val); break;
    case NRF_WIFI_ASSERT_GREATER_THAN_EQUAL_TO: pass = (test_val >= val); break;
    default: break;
    }

    if (!pass) {
        printf("[nrf70 ASSERT] %s (test=%d val=%d op=%d)\r\n",
               msg ? msg : "?", test_val, val, op);
        /* Halt in debug; in production you may choose to reset */
        while (1) { /* hang */ }
    }
}

static unsigned int stm32_strlen(const void *str)
{
    return (unsigned int)strlen((const char *)str);
}

/** Simple 8-bit LFSR random number generator (no RNG peripheral required) */
static unsigned char stm32_rand8_get(void)
{
    static uint8_t lfsr = 0xACU;
    lfsr = (uint8_t)((lfsr >> 1) ^ (-(lfsr & 1U) & 0xB8U));
    return lfsr;
}

/* -------------------------------------------------------------------------
 * OSAL ops struct
 * -------------------------------------------------------------------------*/

const struct nrf_wifi_osal_ops nrf_wifi_os_bm_ops = {
    /* Memory */
    .mem_alloc  = stm32_mem_alloc,
    .mem_zalloc = stm32_mem_zalloc,
    .mem_free   = free,
    .mem_cpy    = stm32_mem_cpy,
    .mem_set    = stm32_mem_set,
    .mem_cmp    = stm32_mem_cmp,

    /* QSPI-named SPI bus ops */
    .qspi_read_reg32  = stm32_qspi_read_reg32,
    .qspi_write_reg32 = stm32_qspi_write_reg32,
    .qspi_cpy_from    = stm32_qspi_cpy_from,
    .qspi_cpy_to      = stm32_qspi_cpy_to,

    /* SPI-named ops */
    .spi_read_reg32  = stm32_spi_read_reg32,
    .spi_write_reg32 = stm32_spi_write_reg32,
    .spi_cpy_from    = stm32_spi_cpy_from,
    .spi_cpy_to      = stm32_spi_cpy_to,

    /* Spinlocks */
    .spinlock_alloc   = stm32_spinlock_alloc,
    .spinlock_free    = stm32_spinlock_free,
    .spinlock_init    = stm32_spinlock_init,
    .spinlock_take    = stm32_spinlock_take,
    .spinlock_rel     = stm32_spinlock_rel,
    .spinlock_irq_take = stm32_spinlock_irq_take,
    .spinlock_irq_rel  = stm32_spinlock_irq_rel,

    /* Logging */
    .log_dbg  = stm32_log_dbg,
    .log_info = stm32_log_info,
    .log_err  = stm32_log_err,

    /* Linked lists */
    .llist_node_alloc    = stm32_llist_node_alloc,
    .llist_node_free     = stm32_llist_node_free,
    .llist_node_data_get = stm32_llist_node_data_get,
    .llist_node_data_set = stm32_llist_node_data_set,
    .llist_alloc         = stm32_llist_alloc,
    .llist_free          = stm32_llist_free,
    .llist_init          = stm32_llist_init,
    .llist_add_node_tail = stm32_llist_add_node_tail,
    .llist_add_node_head = stm32_llist_add_node_head,
    .llist_get_node_head = stm32_llist_get_node_head,
    .llist_get_node_nxt  = stm32_llist_get_node_nxt,
    .llist_del_node      = stm32_llist_del_node,
    .llist_len           = stm32_llist_len,

    /* Network buffers (scan-only mode) */
#ifdef CONFIG_NRF70_BM_SCAN_ONLY
    .nbuf_alloc          = stm32_nbuf_alloc,
    .nbuf_free           = stm32_nbuf_free,
    .nbuf_headroom_res   = stm32_nbuf_headroom_res,
    .nbuf_headroom_get   = stm32_nbuf_headroom_get,
    .nbuf_data_size      = stm32_nbuf_data_size,
    .nbuf_data_get       = stm32_nbuf_data_get,
    .nbuf_data_put       = stm32_nbuf_data_put,
    .nbuf_data_push      = stm32_nbuf_data_push,
    .nbuf_data_pull      = stm32_nbuf_data_pull,
    .nbuf_get_priority   = stm32_nbuf_get_priority,
    .nbuf_get_chksum_done = stm32_nbuf_get_chksum_done,
    .nbuf_set_chksum_done = stm32_nbuf_set_chksum_done,
#endif

    /* Tasklets */
    .tasklet_alloc    = stm32_tasklet_alloc,
    .tasklet_free     = stm32_tasklet_free,
    .tasklet_init     = stm32_tasklet_init,
    .tasklet_schedule = stm32_tasklet_schedule,
    .tasklet_kill     = stm32_tasklet_kill,

    /* Timing */
    .sleep_ms          = stm32_sleep_ms,
    .delay_us          = stm32_delay_us,
    .time_get_curr_us  = stm32_time_get_curr_us,
    .time_elapsed_us   = stm32_time_elapsed_us,
    .time_get_curr_ms  = stm32_time_get_curr_ms,
    .time_elapsed_ms   = stm32_time_elapsed_ms,

    /* QSPI-named bus init/deinit/add/rem */
    .bus_qspi_init               = stm32_bus_qspi_init,
    .bus_qspi_deinit             = stm32_bus_qspi_deinit,
    .bus_qspi_dev_add            = stm32_bus_qspi_dev_add,
    .bus_qspi_dev_rem            = stm32_bus_qspi_dev_rem,
    .bus_qspi_dev_init           = stm32_bus_qspi_dev_init,
    .bus_qspi_dev_deinit         = stm32_bus_qspi_dev_deinit,
    .bus_qspi_dev_intr_reg       = stm32_bus_qspi_intr_reg,
    .bus_qspi_dev_intr_unreg     = stm32_bus_qspi_intr_unreg,
    .bus_qspi_dev_host_map_get   = stm32_bus_qspi_dev_host_map_get,

    /* SPI-named bus init/deinit/add/rem */
    .bus_spi_init                = stm32_bus_spi_init,
    .bus_spi_deinit              = stm32_bus_spi_deinit,
    .bus_spi_dev_add             = stm32_bus_spi_dev_add,
    .bus_spi_dev_rem             = stm32_bus_spi_dev_rem,
    .bus_spi_dev_init            = stm32_bus_spi_dev_init,
    .bus_spi_dev_deinit          = stm32_bus_spi_dev_deinit,
    .bus_spi_dev_intr_reg        = stm32_bus_spi_intr_reg,
    .bus_spi_dev_intr_unreg      = stm32_bus_spi_intr_unreg,
    .bus_spi_dev_host_map_get    = stm32_bus_spi_dev_host_map_get,

    /* Power-save ops (only when low-power mode enabled) */
#ifdef CONFIG_NRF_WIFI_LOW_POWER
    .timer_alloc    = stm32_timer_alloc,
    .timer_free     = stm32_timer_free,
    .timer_init     = stm32_timer_init,
    .timer_schedule = stm32_timer_schedule,
    .timer_kill     = stm32_timer_kill,

    .bus_qspi_ps_sleep  = stm32_bus_qspi_ps_sleep,
    .bus_qspi_ps_wake   = stm32_bus_qspi_ps_wake,
    .bus_qspi_ps_status = stm32_bus_qspi_ps_status,
#endif

    /* Misc */
    .assert    = stm32_assert,
    .strlen    = stm32_strlen,
    .rand8_get = stm32_rand8_get,
};
