/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief RPU hardware interface for STM32 bare-metal shim.
 *        Mirrors nrf70_zephyr_shim/source/platform/rpu_hw_if.c.
 *
 * GPIO initialisation, power sequencing (BUCKEN / IOVDD_CTRL), and
 * HOST_IRQ EXTI configuration are implemented here.
 *
 * The HOST_IRQ EXTI ISR must be provided by the application (CubeMX generates
 * EXTI0_IRQHandler / EXTIx_IRQHandler).  Inside HAL_GPIO_EXTI_Callback() the
 * user must call nrf70_host_irq_handler() which is defined below.
 */

#include <stdio.h>
#include <string.h>

#include "autoconf.h"
#include "nrf70_bm_board_config.h"
#include "rpu_hw_if.h"
#include "spi_if.h"

/* -------------------------------------------------------------------------
 * Memory-map table (mirrors Zephyr rpu_hw_if.c)
 * -------------------------------------------------------------------------*/
char blk_name[][15] = {
    "SysBus", "ExtSysBus", "PBus",     "PKTRAM",
    "GRAM",   "LMAC_ROM",  "LMAC_RET_RAM", "LMAC_SRC_RAM",
    "UMAC_ROM", "UMAC_RET_RAM", "UMAC_SRC_RAM"
};

uint32_t rpu_7002_memmap[][3] = {
    { 0x000000, 0x008FFF, 1 },
    { 0x009000, 0x03FFFF, 2 },
    { 0x040000, 0x07FFFF, 1 },
    { 0x0C0000, 0x0F0FFF, 0 },
    { 0x080000, 0x092000, 1 },
    { 0x100000, 0x134000, 1 },
    { 0x140000, 0x14C000, 1 },
    { 0x180000, 0x190000, 1 },
    { 0x200000, 0x261800, 1 },
    { 0x280000, 0x2A4000, 1 },
    { 0x300000, 0x338000, 1 }
};

/* -------------------------------------------------------------------------
 * IRQ callback storage
 * -------------------------------------------------------------------------*/
static void *g_irq_callbk_data;
static int (*g_irq_callbk_fn)(void *callbk_data);

/**
 * @brief Called by the application's HAL_GPIO_EXTI_Callback() when
 *        HOST_IRQ fires.  Must be called from the EXTI ISR.
 */
void nrf70_host_irq_handler(void)
{
    if (g_irq_callbk_fn != NULL) {
        g_irq_callbk_fn(g_irq_callbk_data);
    }
}

/* -------------------------------------------------------------------------
 * Address validation
 * -------------------------------------------------------------------------*/
static int validate_addr_blk(uint32_t start_addr, uint32_t end_addr,
                              uint32_t block_no, bool *hl_flag,
                              int *selected_blk)
{
    uint32_t *bm = rpu_7002_memmap[block_no];

    if ((start_addr >= bm[0] && start_addr <= bm[1]) &&
        (end_addr   >= bm[0] && end_addr   <= bm[1])) {
        if (block_no == PKTRAM) {
            *hl_flag = 0;
        }
        *selected_blk = block_no;
        return 0;
    }
    return -1;
}

static int rpu_validate_addr(uint32_t start_addr, uint32_t len, bool *hl_flag)
{
    int selected_blk = -1;
    uint32_t end_addr = start_addr + len - 1;
    int i;

    *hl_flag = 1;

    for (i = 0; i < NUM_MEM_BLOCKS; i++) {
        if (validate_addr_blk(start_addr, end_addr, i, hl_flag, &selected_blk) == 0) {
            break;
        }
    }

    if (selected_blk < 0) {
        printf("[nrf70 rpu] Address validation failed: 0x%08x len=%u\r\n",
               start_addr, len);
        return -1;
    }

    if (selected_blk == LMAC_ROM || selected_blk == UMAC_ROM) {
        printf("[nrf70 rpu] Error: cannot write to ROM block\r\n");
        return -1;
    }

    /* Update slave latency based on block */
    g_spi_slave_latency = (*hl_flag) ? (unsigned char)rpu_7002_memmap[selected_blk][2] : 0;

    return 0;
}

/* -------------------------------------------------------------------------
 * GPIO helpers
 * -------------------------------------------------------------------------*/
static int rpu_gpio_config(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* BUCKEN — output push-pull */
    gpio.Pin   = NRF70_BUCKEN_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(NRF70_BUCKEN_PORT, &gpio);
    HAL_GPIO_WritePin(NRF70_BUCKEN_PORT, NRF70_BUCKEN_PIN, GPIO_PIN_RESET);

    /* IOVDD_CTRL — output push-pull */
    gpio.Pin = NRF70_IOVDD_CTRL_PIN;
    HAL_GPIO_Init(NRF70_IOVDD_CTRL_PORT, &gpio);
    HAL_GPIO_WritePin(NRF70_IOVDD_CTRL_PORT, NRF70_IOVDD_CTRL_PIN, GPIO_PIN_RESET);

    /* CS — output push-pull, starts deasserted (HIGH) */
    gpio.Pin = NRF70_CS_PIN;
    HAL_GPIO_Init(NRF70_CS_PORT, &gpio);
    HAL_GPIO_WritePin(NRF70_CS_PORT, NRF70_CS_PIN, GPIO_PIN_SET);

    return 0;
}

static int rpu_gpio_remove(void)
{
    HAL_GPIO_WritePin(NRF70_BUCKEN_PORT,   NRF70_BUCKEN_PIN,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF70_IOVDD_CTRL_PORT, NRF70_IOVDD_CTRL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_DeInit(NRF70_BUCKEN_PORT,     NRF70_BUCKEN_PIN);
    HAL_GPIO_DeInit(NRF70_IOVDD_CTRL_PORT, NRF70_IOVDD_CTRL_PIN);
    HAL_GPIO_DeInit(NRF70_CS_PORT,         NRF70_CS_PIN);
    return 0;
}

static int rpu_pwron(void)
{
    /* Assert BUCKEN, wait for buck regulator to settle */
    HAL_GPIO_WritePin(NRF70_BUCKEN_PORT, NRF70_BUCKEN_PIN, GPIO_PIN_SET);
    HAL_Delay(NRF70_BUCKEN_SETTLE_MS);

    /* Assert IOVDD_CTRL, wait for IO voltage to settle */
    HAL_GPIO_WritePin(NRF70_IOVDD_CTRL_PORT, NRF70_IOVDD_CTRL_PIN, GPIO_PIN_SET);
    HAL_Delay(NRF70_IOVDD_SETTLE_MS);

    return 0;
}

static int rpu_pwroff(void)
{
    HAL_GPIO_WritePin(NRF70_BUCKEN_PORT,     NRF70_BUCKEN_PIN,     GPIO_PIN_RESET);
    HAL_GPIO_WritePin(NRF70_IOVDD_CTRL_PORT, NRF70_IOVDD_CTRL_PIN, GPIO_PIN_RESET);
    return 0;
}

/* -------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------*/

int rpu_irq_config(void *callbk_data, int (*callbk_fn)(void *callbk_data))
{
    GPIO_InitTypeDef gpio = {0};

    g_irq_callbk_data = callbk_data;
    g_irq_callbk_fn   = callbk_fn;

    /* Configure HOST_IRQ pin as input with rising-edge EXTI */
    gpio.Pin  = NRF70_IRQ_PIN;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(NRF70_IRQ_PORT, &gpio);

    /* The NVIC IRQ enable for the EXTI line must be done in CubeMX-generated
     * MX_GPIO_Init() or by the user calling HAL_NVIC_EnableIRQ() after this
     * function returns. */

    printf("[nrf70 rpu] IRQ configured on pin 0x%04x\r\n", NRF70_IRQ_PIN);
    return 0;
}

int rpu_irq_remove(void)
{
    HAL_GPIO_DeInit(NRF70_IRQ_PORT, NRF70_IRQ_PIN);
    g_irq_callbk_fn   = NULL;
    g_irq_callbk_data = NULL;
    return 0;
}

int rpu_read(unsigned int addr, void *data, int len)
{
    bool hl_flag;

    if (rpu_validate_addr(addr, len, &hl_flag)) {
        return -1;
    }

    if (hl_flag) {
        return spim_hl_read(addr, data, len);
    } else {
        return spim_read(addr, data, len);
    }
}

int rpu_write(unsigned int addr, const void *data, int len)
{
    bool hl_flag;

    if (rpu_validate_addr(addr, len, &hl_flag)) {
        return -1;
    }

    return spim_write(addr, data, len);
}

int rpu_wrsr2(uint8_t data)
{
    return spim_cmd_rpu_wakeup_fn(data);
}

int rpu_rdsr2(void)
{
    return spi_validate_rpu_wake_writecmd();
}

int rpu_rdsr1(void)
{
    return spim_wait_while_rpu_awake();
}

int rpu_sleep(void)
{
    return spim_cmd_sleep_rpu_fn();
}

int rpu_wakeup(void)
{
    int ret;

    ret = rpu_wrsr2(1);
    if (ret) {
        printf("[nrf70 rpu] WRSR2 failed\r\n");
        return ret;
    }

    ret = rpu_rdsr2();
    if (ret < 0) {
        printf("[nrf70 rpu] RDSR2 failed\r\n");
        return ret;
    }

    ret = rpu_rdsr1();
    if (ret < 0) {
        printf("[nrf70 rpu] RDSR1 failed\r\n");
        return ret;
    }

    return 0;
}

int rpu_sleep_status(void)
{
    int ret = rpu_rdsr1();
    if (ret < 0) {
        return 0;  /* Assume sleeping if can't read */
    }
    return ret;
}

int rpu_clks_on(void)
{
    uint32_t rpu_clks = 0x100;
    return rpu_write(0x048C20, &rpu_clks, 4);
}

int rpu_qspi_read_reg(uint8_t reg_addr, uint8_t *reg_value)
{
    return spim_read_reg(reg_addr, reg_value);
}

int rpu_qspi_write_reg(uint8_t reg_addr, uint8_t reg_value)
{
    return spim_write_reg(reg_addr, reg_value);
}

int rpu_init(void)
{
    int ret;

    ret = rpu_gpio_config();
    if (ret) {
        return ret;
    }

    ret = spim_init();
    if (ret) {
        rpu_gpio_remove();
        return ret;
    }

    ret = rpu_pwron();
    if (ret) {
        spim_deinit();
        rpu_gpio_remove();
        return ret;
    }

    return 0;
}

int rpu_enable(void)
{
    int ret;

    ret = rpu_wakeup();
    if (ret) {
        rpu_pwroff();
        return ret;
    }

    ret = rpu_clks_on();
    if (ret) {
        rpu_pwroff();
        return ret;
    }

    return 0;
}

int rpu_disable(void)
{
    rpu_pwroff();
    spim_deinit();
    rpu_gpio_remove();
    rpu_irq_remove();
    return 0;
}
