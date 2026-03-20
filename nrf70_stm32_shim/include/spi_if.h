/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief SPI interface prototypes for the STM32 bare-metal nRF70 shim.
 *        Mirrors nrf70_zephyr_shim/include/spi_if.h but uses STM32 HAL.
 */

#ifndef __STM32_SPI_IF_H__
#define __STM32_SPI_IF_H__

#include <stdint.h>

/* Slave-latency setting (0 = no extra latency, 1 = one extra word) */
extern unsigned char g_spi_slave_latency;

/* Initialise / de-initialise the SPI bus */
int spim_init(void);
int spim_deinit(void);

/* Bulk read/write of RPU memory via 24-bit address */
int spim_write(unsigned int addr, const void *data, int len);
int spim_read(unsigned int addr, void *data, int len);

/* High-latency read (addr < 0x0C0000 — uses extra discard words) */
int spim_hl_read(unsigned int addr, void *data, int len);

/* Single-byte register access (WRSR2 / RDSR1 / RDSR2) */
int spim_read_reg(uint8_t reg_addr, uint8_t *reg_value);
int spim_write_reg(uint8_t reg_addr, uint8_t reg_value);

/* RPU wake / sleep via WRSR2 register */
int spim_cmd_rpu_wakeup_fn(uint32_t data);
int spim_cmd_sleep_rpu_fn(void);

/* Poll RDSR1 until RPU_AWAKE bit is set (or timeout) */
int spim_wait_while_rpu_awake(void);

/* Poll RDSR2 until RPU_WAKEUP_NOW write is acknowledged */
int spi_validate_rpu_wake_writecmd(void);

#endif /* __STM32_SPI_IF_H__ */
